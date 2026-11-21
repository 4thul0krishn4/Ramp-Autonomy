#!/usr/bin/env python3
"""The simulated IMU has NO noise model. This adds a synthetic constant gyro-z bias (worst case, applied in the
direction that hurts) to the recorded signal and reports, for the chosen rule (2 s window, 0.10 rad), how the
detector's two error modes move:
  blocked windows vetoed  -> a real freeze is masked (missed detection) - the SAFETY-relevant error
  moving-rotating windows NOT suppressed -> a false latch survives
Typical MEMS gyro bias is ~0.005-0.02 rad/s (0.3-1 deg/s); consumer parts drift more.
Usage: gyro_bias_robustness.py <session.npz> [...]"""
import sys, math
import numpy as np
from analyze_events import Session, commanded_at

TRAPZ = getattr(np, 'trapezoid', None) or np.trapz
W, TH = 2.0, 0.10
BIASES = [0.0, 0.005, 0.01, 0.02, 0.03, 0.05]


def main():
    blocked, rot = [], []
    for f in sys.argv[1:]:
        S = Session(f)
        cage = [(e['t_done'] + 2.0, e['t_obs_end']) for e in S.events if e.get('ok') and 't_obs_end' in e]
        it, wz = S.imu[:, 0], S.imu[:, 7]
        for t in np.arange(S.tt[0] + 6, S.tt[-1] - 0.5, 0.25):
            if not commanded_at(S, t):
                continue
            m = (it >= t - W) & (it <= t)
            if m.sum() < 5:
                continue
            x1, y1, a1 = S.pose(t); x0, y0, a0 = S.pose(t - 2.0)
            disp, dyaw = math.hypot(x1 - x0, y1 - y0), abs(a1 - a0)
            true_int = float(TRAPZ(wz[m], it[m]))
            if any(a <= t <= b for a, b in cage) and disp < 0.06:
                blocked.append(true_int)
            elif dyaw >= 0.10:
                rot.append(true_int)
    blocked, rot = np.array(blocked), np.array(rot)
    print(f"windows: blocked n={len(blocked)}   moving-rotating n={len(rot)}   rule: |integral over {W:.0f} s| >= {TH} rad vetoes the latch")
    print(f"{'gyro bias':>10s} {'integral bias':>14s} {'blocked windows vetoed (freeze MASKED)':>40s} {'rotating windows still latching':>34s}")
    for b in BIASES:
        # worst case: the bias adds |b|*W to the magnitude of the integral for blocked windows, and subtracts it for rotating ones
        veto = np.mean(np.abs(blocked) + b * W >= TH) * 100
        miss = np.mean(np.abs(rot) - b * W < TH) * 100
        print(f"{b:8.3f}   {b * W:12.3f}   {veto:38.1f}%   {miss:32.1f}%")


if __name__ == '__main__':
    main()
