#!/usr/bin/env python3
"""Aggregate recorded sessions into the final old-vs-new detector comparison, all against Gazebo ground truth.
Usage: aggregate.py <label> [<label> ...]      (reads data/<label>_{normal,freeze}_*.npz)

NORMAL sessions (no fixture) -> false-positive behaviour:
    latches, latches while the robot was MOVING (false), by phase and motion type (rotating / straight),
    seconds spent stall-latched in leg 3, leg-3 duration and completion.
FREEZE sessions (positive control) -> true-positive behaviour:
    per event type, commanded-and-blocked episodes long enough to require a latch (or that were latched):
    detected / scorable, detection latency, and the number of justified latches.
"""
import sys, glob, math, json
import numpy as np
from analyze_events import Session, score_event, justified
from replay_detectors import phase, rotating_at


def wilson(k, n, z=1.96):
    if n == 0:
        return float('nan'), float('nan')
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def stalled_seconds(S, t0, t1):
    s = S.stalled
    tot = 0.0
    for i in range(len(s)):
        if s[i, 1] == 1:
            a = s[i, 0]; b = s[i + 1, 0] if i + 1 < len(s) else S.t_end
            tot += max(0.0, min(b, t1) - max(a, t0))
    return tot


def normal_report(label):
    files = sorted(glob.glob(f'data/{label}_normal_*.npz'))
    rows = []
    for f in files:
        S = Session(f)
        ex = S.extra; r = ex['result']
        if ex['health'].get('starved'):
            print(f"  (skipping STARVED session {f})"); continue
        t2, t3 = r['t_leg2_end'], r['t_leg3_end']
        by = {}
        for e in S.edges:
            if not S.moving_at(e):
                continue
            k = phase(S, e) + (' rotating' if rotating_at(S, e) else ' straight')
            by[k] = by.get(k, 0) + 1
        l3 = [e for e in S.edges if t2 <= e <= t3]
        rows.append(dict(file=f.split('/')[-1], payload=ex['payload'], leg3_s=t3 - t2, ok=r['leg3_ok'],
                         latches=len(S.edges), false=sum(S.moving_at(e) for e in S.edges),
                         l3_latches=len(l3), l3_false=sum(S.moving_at(e) for e in l3),
                         l3_stalled_s=stalled_seconds(S, t2, t3), by=by))
    return rows


def freeze_report(label):
    files = sorted(glob.glob(f'data/{label}_freeze_*.npz'))
    per = {}
    just = 0; sess = 0
    for f in files:
        S = Session(f)
        if S.extra['health'].get('starved'):
            print(f"  (skipping STARVED session {f})"); continue
        sess += 1
        for ev in S.events:
            r = score_event(S, ev)
            d = per.setdefault(ev['kind'], dict(det=0, scor=0, lat=[], eps=0, spawn_fail=0))
            if not ev['ok']:
                d['spawn_fail'] += 1; continue
            d['eps'] += r['n_episodes']; d['det'] += r['n_detected']; d['scor'] += r['n_scorable']
            d['lat'] += [e['latch_latency'] for e in r['episodes'] if e['detected']]
        just += sum(justified(S, e) for e in S.edges)
    return sess, per, just


def main():
    for label in sys.argv[1:]:
        print(f"\n################ detector: {label} ################")
        rows = normal_report(label)
        if rows:
            print(f"NORMAL sessions (n={len(rows)}): every latch is scored against ground truth")
            print(f"  {'session':34s} {'leg3 s':>7s} {'ok':>3s} {'latches':>8s} {'false':>6s} {'leg3 latches':>13s} {'leg3 false':>11s} {'leg3 stalled s':>15s}")
            for r in rows:
                print(f"  {r['file']:34s} {r['leg3_s']:7.0f} {str(r['ok'])[0]:>3s} {r['latches']:8d} {r['false']:6d} {r['l3_latches']:13d} {r['l3_false']:11d} {r['l3_stalled_s']:15.0f}")
            m = lambda k: (np.mean([r[k] for r in rows]), np.std([r[k] for r in rows], ddof=1) if len(rows) > 1 else 0.0)
            for k, name in (('leg3_s', 'leg-3 duration (s)'), ('latches', 'latches / session'), ('false', 'FALSE latches / session'),
                            ('l3_false', 'FALSE latches in leg 3 / session'), ('l3_stalled_s', 'seconds stall-latched in leg 3')):
                a, b = m(k); print(f"  mean {name:36s} {a:7.1f}  (sd {b:.1f})")
            fr = sum(r['l3_stalled_s'] for r in rows) / sum(r['leg3_s'] for r in rows)
            print(f"  fraction of leg-3 time spent stall-latched: {100 * fr:.1f}%   completed: {sum(r['ok'] for r in rows)}/{len(rows)}")
            tot = {}
            for r in rows:
                for k, v in r['by'].items():
                    tot[k] = tot.get(k, 0) + v
            print("  FALSE latches by phase / motion type (all sessions): " + ', '.join(f'{k}: {v}' for k, v in sorted(tot.items())))
        sess, per, just = freeze_report(label)
        if per:
            print(f"FREEZE sessions (n={sess}); positive control = commanded-and-blocked episodes (ground truth)")
            print(f"  {'event':14s} {'detected/scorable':>18s} {'95% Wilson CI':>16s} {'latency s (median / max)':>26s}")
            D = S_ = 0
            for kind in ('straight_raw', 'arc', 'taper'):
                if kind not in per:
                    continue
                d = per[kind]; lo, hi = wilson(d['det'], d['scor']); D += d['det']; S_ += d['scor']
                lat = d['lat']
                ls = f"{np.median(lat):.1f} / {max(lat):.1f}" if lat else '-'
                print(f"  {kind:14s} {d['det']:>10d}/{d['scor']:<7d} {100 * lo:6.0f}-{100 * hi:.0f}% {ls:>26s}   (spawn failures {d['spawn_fail']})")
            lo, hi = wilson(D, S_)
            print(f"  ALL            {D:>10d}/{S_:<7d} {100 * lo:6.0f}-{100 * hi:.0f}%      justified latches total: {just}")


if __name__ == '__main__':
    main()
