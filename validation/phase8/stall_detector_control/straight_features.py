#!/usr/bin/env python3
"""Can any independent (non-wheel, non-LiDAR) IMU feature tell straight travel from a blocked chassis?

Rotation is observable from the gyro. For straight-line translation on the featureless platform the
LiDAR check is blind, so this asks whether the IMU carries ANY discriminating signal, measured with
ground truth. Windows (3 s, 0.5 s grid):
  MOVING-STRAIGHT : ground truth travelled >= 0.5 m, turned < 0.05 rad, command nonzero throughout
  BLOCKED         : command nonzero throughout, ground truth blocked (<6 cm / 6 deg)
Per feature: AUC = P(feature(MOVING) > feature(BLOCKED)); 0.5 = no information, 1.0 or 0.0 = separable
(direction irrelevant, so |AUC-0.5| matters). Usage: straight_features.py <session.npz> [...]
"""
import sys, math
import numpy as np
from analyze_events import Session, commanded_at

W = 3.0


def auc(a, b):
    if len(a) < 3 or len(b) < 3:
        return float('nan')
    a, b = np.asarray(a), np.asarray(b)
    return float(((a[:, None] > b[None, :]).sum() + 0.5 * (a[:, None] == b[None, :]).sum()) / (len(a) * len(b)))


def feats(S, t):
    m = (S.imu[:, 0] >= t - W) & (S.imu[:, 0] <= t)
    I = S.imu[m]
    if len(I) < 50:
        return None
    ax, ay, az = I[:, 8], I[:, 9], I[:, 10]
    gx, gy, gz = I[:, 5], I[:, 6], I[:, 7]
    pitch = np.arcsin(np.clip(2 * (I[:, 4] * I[:, 2] - I[:, 3] * I[:, 1]), -1, 1))
    roll = np.arctan2(2 * (I[:, 4] * I[:, 1] + I[:, 2] * I[:, 3]), 1 - 2 * (I[:, 1] ** 2 + I[:, 2] ** 2))
    dt = np.diff(I[:, 0], prepend=I[0, 0])
    v_int = abs(float(np.sum((ax - ax.mean()) * dt)))      # drift-corrected integrated forward accel
    return dict(std_ax=ax.std(), std_ay=ay.std(), std_az=az.std(), std_gx=gx.std(), std_gy=gy.std(),
                mean_abs_gz=abs(gz.mean()), pitch_mean=pitch.mean(), pitch_std=pitch.std(), roll_std=roll.std(),
                mean_ax=ax.mean(), hf_ax=np.abs(np.diff(ax)).mean(), hf_az=np.abs(np.diff(az)).mean(), v_int=v_int)


def main():
    mov, blk = [], []
    for p in sys.argv[1:]:
        S = Session(p)
        r = S.extra.get('result') or {}
        cage = [(e['t_done'], e['t_obs_end']) for e in S.events if e.get('ok') and 't_obs_end' in e]
        for t in np.arange(S.tt[0] + W + 1, S.tt[-1] - 0.5, 0.5):
            ts = np.arange(t - W, t + 1e-9, 0.1)
            if not all(commanded_at(S, u) for u in ts):
                continue
            xs = np.array([S.pose(u) for u in ts])
            dist = math.hypot(xs[-1, 0] - xs[0, 0], xs[-1, 1] - xs[0, 1])
            dyaw = abs(xs[-1, 2] - xs[0, 2])
            incage = any(a <= t <= b for a, b in cage)
            f = feats(S, t)
            if f is None:
                continue
            if dist >= 0.5 and dyaw < 0.05:
                mov.append(f)
            elif incage and np.hypot(xs[:, 0] - xs[0, 0], xs[:, 1] - xs[0, 1]).max() < 0.06 and np.abs(xs[:, 2] - xs[0, 2]).max() < np.radians(6):
                blk.append(f)
    print(f"windows: MOVING-STRAIGHT n={len(mov)}   BLOCKED n={len(blk)}")
    if len(mov) < 3 or len(blk) < 3:
        print("not enough windows of one class to score"); return
    print(f"{'feature':12s} {'MOVING median':>14s} {'BLOCKED median':>15s} {'AUC':>6s}   verdict")
    for k in mov[0]:
        a, b = [f[k] for f in mov], [f[k] for f in blk]
        u = auc(a, b)
        sep = max(u, 1 - u)
        print(f"{k:12s} {np.median(a):14.5f} {np.median(b):15.5f} {u:6.2f}   "
              f"{'SEPARABLE' if sep >= 0.95 else ('weak' if sep >= 0.75 else 'no information')}")


if __name__ == '__main__':
    main()
