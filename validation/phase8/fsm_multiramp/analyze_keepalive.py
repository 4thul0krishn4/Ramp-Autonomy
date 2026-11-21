#!/usr/bin/env python3
"""Per-arm analysis of the keepalive isolation experiment: HALT cadence vs the switcher's re-push cadence vs MPPI optimizer-reset cadence.
Usage: analyze_keepalive.py <outdir> <arm> [<arm> ...]   (reads <arm>.json from the probe and <arm>_launch.log from the guarded run)"""
import bisect, collections, json, re, sys, statistics as st
d = sys.argv[1]
def epoch(line):
    m = re.search(r'\[(\d{10}\.\d+)\]', line); return float(m.group(1)) if m else None
for arm in sys.argv[2:]:
    try: r = json.load(open(f'{d}/{arm}.json'))
    except Exception as e: print(f'== {arm}: no data ({e})'); continue
    log = open(f'{d}/{arm}_launch.log').read().splitlines(); t0 = r['t0_epoch']
    pushes = [epoch(l) - t0 for l in log if 'confirmed MPPI profile' in l and 'nav2_param_switcher' in l]
    resets = sorted(epoch(l) - t0 for l in log if 'Optimizer reset' in l)
    bursts = ([resets[0]] + [b for a, b in zip(resets[:-1], resets[1:]) if b - a > 0.5]) if resets else []
    imu = r['imu']; ts = [x[0] for x in imu]
    halts = []
    for t in r['transitions']:
        if t['to'] != 'HALT': continue
        i = bisect.bisect_left(ts, t['t'] - 0.15); j = bisect.bisect_right(ts, t['t'] + 0.05); w = imu[i:j]
        slip = max(x[3] for x in w); stall = max(x[4] for x in w)
        halts.append((t['t'], t['frm'], 'slip' if slip > 0.5 else ('stall' if stall else 'other')))
    slip_h = [h for h in halts if h[2] == 'slip' and h[1] in ('TRANSITION', 'ASCENT')]
    gaps = [round(b[0] - a[0], 1) for a, b in zip(slip_h[:-1], slip_h[1:])]
    pg = [round(b - a, 1) for a, b in zip(pushes[:-1], pushes[1:])]
    bg = [round(b - a, 1) for a, b in zip(bursts[:-1], bursts[1:])]
    print(f'== arm {arm}: legs {[(l["name"], l["ok"]) for l in r["legs"]]}  duration {r["legs"][-1]["t1"] if r["legs"] else None} s')
    print(f'   HALT entries: {len(halts)} total, causes {dict(collections.Counter(h[2] for h in halts))}; slip-HALTs during TRANSITION/ASCENT climbs: {len(slip_h)}')
    print(f'   gaps between those slip-HALTs (s, rounded 0.1): most common {collections.Counter(gaps).most_common(5)}   median {st.median(gaps) if gaps else None}')
    print(f'   switcher pushes ("confirmed MPPI profile"): {len(pushes)}; gaps most common {collections.Counter(pg).most_common(4)}')
    print(f'   MPPI "Optimizer reset" bursts: {len(bursts)} ({len(resets)} resets); gaps most common {collections.Counter(bg).most_common(4)}')
    if slip_h and bursts:
        lags = []
        for h in slip_h:
            k = bisect.bisect_right(bursts, h[0]) - 1
            if k >= 0: lags.append(h[0] - bursts[k])
        print(f'   lag from the most recent optimizer-reset burst to each slip-HALT: n={len(lags)} median {st.median(lags):.2f} s, p10 {sorted(lags)[len(lags)//10]:.2f}, p90 {sorted(lags)[(9*len(lags))//10]:.2f}')
    # what the wheels were told around the resets (cmd_vel actuation): min |vx| in [-0.3,+0.6] s around each burst vs elsewhere on the same legs
    cmd = r.get('cmd') or []
    if cmd and bursts:
        ct = [c[0] for c in cmd]; near = []; 
        for b in bursts:
            i = bisect.bisect_left(ct, b - 0.1); j = bisect.bisect_right(ct, b + 0.6)
            if j > i: near.append(min(abs(c[1]) for c in cmd[i:j]))
        allv = [abs(c[1]) for c in cmd if abs(c[1]) > 0]
        print(f'   actuation cmd_vel: min |vx| within 0.6 s after a reset burst: median {st.median(near):.3f} m/s (n={len(near)}); overall median |vx| {st.median(allv):.3f}')
