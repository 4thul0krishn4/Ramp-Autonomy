#!/usr/bin/env python3
"""Offline replay of motion_stall_detector_node's LiDAR stall logic (and candidate variants) over
sessions recorded by freeze_session.py, scored against Gazebo ground truth.

  old         : the node's current logic, verbatim (scan-consistency only)
  yaw(W,th)   : same, but a scan verdict of 'no movement' is overridden when the IMU orientation yaw
                changed by >= th rad over the last W seconds (rotation observed independently of the
                wheels and of the LiDAR)

Two views:
  * simulated latches  - the node's full state machine (5 s commanded clock, latch, 10 s cooldown) run
                         open-loop on the recorded inputs; the 'old' variant is checked against the
                         latches the live node actually produced (replay fidelity).
  * per-tick condition - at every scan tick where the actuation command has been continuously nonzero
                         for >= 5 s, is the 'stall' condition true?  Scored on ticks where ground truth
                         says the robot is MOVING (false-positive rate) and on ticks inside a VERIFIED
                         freeze (true-positive rate).  This is independent of closed-loop artefacts.
Usage: replay_detectors.py <session.npz> [...]
"""
import sys, json, math
import numpy as np
from analyze_events import Session

WINDOW, COOLDOWN, THR, MINB, DEBOUNCE, CMD_MIN = 5.0, 10.0, 0.05, 30, 0.5, 0.05


def scan_diff(scans, times, now):
    i = np.searchsorted(times, now, side='right') - 1
    j = np.searchsorted(times, now - WINDOW, side='right') - 1
    if i < 0 or j < 0 or i == j:
        return 0, None
    a, b = scans[i], scans[j]
    ok = np.isfinite(a) & np.isfinite(b)
    n = int(ok.sum())
    return (n, float(np.abs(a[ok] - b[ok]).mean())) if n else (0, None)


class Variant:
    def __init__(self, name, yaw_w=None, yaw_th=None):
        self.name, self.yaw_w, self.yaw_th = name, yaw_w, yaw_th


def imu_yaw(S):
    """cumulative gyro-z integral (same quantity the node integrates over its window)"""
    t, wz = S.imu[:, 0], S.imu[:, 7]
    y = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(t))])
    return t, y


def run(S, z, var):
    """returns (edges, ticks) where ticks = [(t, cond_true, commanded_ge5)]"""
    st, sc = z['scan_t'], z['scan']
    s3t, s3 = z['scan360_t'], z['scan360']
    it, iy = imu_yaw(S)
    act = S.act
    tick_times = np.sort(np.r_[st, s3t])
    cmd_since, below_since = None, None
    latched, latched_since = False, None
    ai = 0
    edges, ticks = [], []
    prev = False
    for now in tick_times:
        while ai < len(act) and act[ai, 0] <= now:
            t, vx, wz = act[ai]
            if abs(vx) >= CMD_MIN or abs(wz) >= CMD_MIN:
                below_since = None
                if cmd_since is None:
                    cmd_since = t
            else:
                if below_since is None:
                    below_since = t
                elif t - below_since >= DEBOUNCE:
                    cmd_since = None
            ai += 1
        n3, d3 = scan_diff(s3, s3t, now)
        nr, dr = scan_diff(sc, st, now)
        if n3 >= MINB:
            no_move, have = d3 < THR, True
        elif nr >= MINB:
            no_move, have = dr < THR, True
        else:
            no_move, have = False, False
        if var.yaw_w is not None and no_move:
            a = np.interp(now, it, iy); b = np.interp(now - var.yaw_w, it, iy)
            if abs(a - b) >= var.yaw_th:
                no_move = False            # rotation observed by the IMU: chassis is not frozen
        commanded5 = cmd_since is not None and now - cmd_since >= WINDOW
        ticks.append((now, bool(no_move and have and commanded5), commanded5))
        # --- the node's latch logic ---
        result = None
        if latched:
            if have and not no_move:
                latched, latched_since, result = False, None, False
            elif latched_since is not None and now - latched_since >= COOLDOWN:
                latched, latched_since, result = False, None, False
            else:
                result = True
        else:
            if cmd_since is None or now - cmd_since < WINDOW or not have:
                result = False
            else:
                latched = bool(no_move)
                if latched:
                    latched_since = now
                result = latched
        if result and not prev:
            edges.append(float(now))
        prev = bool(result)
    return edges, ticks


def phase(S, t):
    r = S.extra.get('result') or {}
    if not r:
        return 'all'
    if t < r['t_leg1_end']: return 'leg1'
    if t < r['t_leg2_end']: return 'leg2'
    if t < r['t_leg3_end']:
        k = np.searchsorted(S.state_t, t, side='right') - 1
        return 'leg3-descent' if k >= 0 and S.state[k] == 'DESCENT' else 'leg3-platform'
    return 'after'


def rotating_at(S, t, span=2.0):
    return abs(S.pose(t)[2] - S.pose(t - span)[2]) >= 0.10


def main():
    from analyze_events import justified
    grid = [(2.0, 0.10), (2.0, 0.15), (1.0, 0.10), (3.0, 0.10)] if '--fast' in sys.argv else \
        [(w, th) for w in (1.0, 2.0, 3.0) for th in (0.05, 0.10, 0.20)]
    variants = [Variant('old')] + [Variant(f'yaw(W={w},th={th})', w, th) for w, th in grid]
    tot = {v.name: dict(ticks_rot=[0, 0], ticks_str=[0, 0], just=[0, 0], moving_edges=[0, 0], sim=[0, 0, 0]) for v in variants}
    for p in [a for a in sys.argv[1:] if not a.startswith('--')]:
        S = Session(p)
        z = np.load(p, allow_pickle=True)
        live_just = [e for e in S.edges if justified(S, e)]
        live_mov = [e for e in S.edges if S.moving_at(e)]
        print(f"\n=== {p.split('/')[-1]} mode={S.extra.get('mode')} payload={S.extra.get('payload')}  "
              f"live latches={len(S.edges)} (moving/false {len(live_mov)}, justified/true {len(live_just)})")
        for var in variants:
            edges, ticks = run(S, z, var)
            tt = np.array([t for t, _, _ in ticks]); cond = np.array([c for _, c, _ in ticks])
            def cond_at(e):
                k = (tt >= e - 0.3) & (tt <= e + 0.3)          # the condition first turns true AT the edge tick
                return bool(cond[k].any()) if k.any() else False
            T = tot[var.name]
            for t, c, c5 in ticks:
                if c5 and S.moving_at(t):
                    d = T['ticks_rot'] if rotating_at(S, t) else T['ticks_str']
                    d[0] += 1; d[1] += c
            for e in edges:
                T['sim'][0] += 1; T['sim'][1] += S.moving_at(e); T['sim'][2] += justified(S, e)
            for e in live_just:
                T['just'][0] += 1; T['just'][1] += cond_at(e)
            for e in live_mov:
                T['moving_edges'][0] += 1; T['moving_edges'][1] += cond_at(e)
        fid = [round(e - S.tt[0], 1) for e in run(S, z, variants[0])[0]]
        live = [round(e - S.tt[0], 1) for e in S.edges]
        print(f"  replay fidelity (old rule vs the live node): live {len(live)} edges, replay {len(fid)} edges")
    print("\n==================== rule comparison over all sessions given ====================")
    print("FP ticks = stall condition TRUE on ticks (commanded >=5 s) where ground truth says the robot is MOVING")
    print("justified kept = true latches (commanded+blocked through the evidence window) still detected by the rule")
    print("old false latches still firing = of the live node's false latches, how many the rule would still make")
    print("sim latches = the node's full latch logic run open-loop on the recorded inputs: total / while MOVING (false) / JUSTIFIED (true)")
    print(f"{'rule':22s} {'FP ticks, ROTATING':>20s} {'FP ticks, STRAIGHT':>20s} {'justified kept':>16s} {'old false latches still firing':>32s} {'sim latches tot/false/true':>27s}")
    for v in variants:
        T = tot[v.name]
        f = lambda d: f"{d[1]}/{d[0]}" + (f" ({100 * d[1] / d[0]:.0f}%)" if d[0] else "")
        print(f"{v.name:22s} {f(T['ticks_rot']):>20s} {f(T['ticks_str']):>20s} {f(T['just']):>16s} {f(T['moving_edges']):>32s} {str(T['sim'][0])+'/'+str(T['sim'][1])+'/'+str(T['sim'][2]):>27s}")


if __name__ == '__main__':
    main()
