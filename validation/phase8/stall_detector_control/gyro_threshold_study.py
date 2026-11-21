#!/usr/bin/env python3
"""Measure the gyro rotation feature (|integrated yaw| over the last W s) on ground-truth-labelled windows:
BLOCKED (commanded, chassis blocked inside the fixture) vs MOVING (rotating / straight). Chooses the
veto threshold from data. Usage: gyro_threshold_study.py <session.npz> [...]"""
import sys, math
import numpy as np
from analyze_events import Session, commanded_at

TRAPZ = getattr(np, 'trapezoid', None) or np.trapz
Ws = [1.0, 2.0, 3.0]


def gyro_int(S, t, W):
    it, wz = S.imu[:, 0], S.imu[:, 7]
    m = (it >= t - W) & (it <= t)
    return None if m.sum() < 5 else abs(float(TRAPZ(wz[m], it[m])))


def q(a):
    a = np.array(a)
    return "n=%d p5=%.3f p50=%.3f p95=%.3f max=%.3f" % (len(a), *np.percentile(a, [5, 50, 95]), a.max()) if len(a) else "n=0"


def main():
    blocked = {k: {W: [] for W in Ws} for k in ('arc', 'taper', 'straight_raw')}
    rot = {W: [] for W in Ws}; strt = {W: [] for W in Ws}
    for f in sys.argv[1:]:
        S = Session(f)
        cage = [(e['kind'], e['t_done'] + 2.0, e['t_obs_end']) for e in S.events if e.get('ok') and 't_obs_end' in e]
        for t in np.arange(S.tt[0] + 6, S.tt[-1] - 0.5, 0.25):
            if not commanded_at(S, t):
                continue
            x1, y1, a1 = S.pose(t); x0, y0, a0 = S.pose(t - 2.0)
            disp, dyaw = math.hypot(x1 - x0, y1 - y0), abs(a1 - a0)
            incage = [k for k, a, b in cage if a <= t <= b]
            for W in Ws:
                g = gyro_int(S, t, W)
                if g is None:
                    continue
                if incage and disp < 0.06:
                    blocked[incage[0]][W].append(g)
                elif disp >= 0.20 or dyaw >= 0.10:
                    (rot if dyaw >= 0.10 else strt)[W].append(g)
    for W in Ws:
        print(f"\n=== gyro |integrated yaw| over the last {W:.0f} s (rad) ===")
        for k in blocked:
            print(f"  BLOCKED {k:13s}: {q(blocked[k][W])}")
        print(f"  MOVING, rotating    : {q(rot[W])}")
        print(f"  MOVING, straight    : {q(strt[W])}")
        print("  th     blocked windows vetoed (=> freeze MISSED): all / arc-fixture      moving-rotating suppressed")
        bl_all = sum((blocked[k][W] for k in blocked), [])
        for th in (0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30):
            ba = 100 * np.mean(np.array(bl_all) >= th) if bl_all else float('nan')
            aa = 100 * np.mean(np.array(blocked['arc'][W]) >= th) if blocked['arc'][W] else float('nan')
            rr = 100 * np.mean(np.array(rot[W]) >= th) if rot[W] else float('nan')
            print(f"  {th:4.2f}   {ba:6.1f}% / {aa:6.1f}%                                   {rr:6.1f}%")


if __name__ == '__main__':
    main()
