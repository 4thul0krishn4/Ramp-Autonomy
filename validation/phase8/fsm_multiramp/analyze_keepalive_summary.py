import bisect, json, re, sys, statistics as st
d = sys.argv[1]; arms = sys.argv[2:]
print(f'{"arm":6s} {"period":>7s} {"climb s":>8s} {"resets in climb":>16s} {"slip-HALTs in climb":>20s} {"HALT/reset":>11s} {"resets with cmd dip <0.02":>26s} {"...of which HALTed":>19s} {"...without dip HALTed":>22s}')
for arm in arms:
    r = json.load(open(f'{d}/{arm}.json')); t0 = r['t0_epoch']
    log = open(f'{d}/{arm}_launch.log').read().splitlines()
    ep = lambda l: float(re.search(r'\[(\d{10}\.\d+)\]', l).group(1)) - t0
    resets = sorted(ep(l) for l in log if 'Optimizer reset' in l and re.search(r'\[(\d{10}\.\d+)\]', l))
    bursts = ([resets[0]] + [b for a, b in zip(resets[:-1], resets[1:]) if b - a > 0.5]) if resets else []
    # climb intervals = time in TRANSITION/ASCENT
    tr = r['transitions']; intervals = []; cur = None; state = 'APPROACH'; last = 0.0
    for t in tr:
        if t['to'] in ('TRANSITION', 'ASCENT') and cur is None: cur = t['t']
        if t['to'] in ('EXIT', 'DESCENT', 'APPROACH') and cur is not None: intervals.append((cur, t['t'])); cur = None
    in_climb = lambda x: any(a <= x <= b for a, b in intervals)
    climb_s = sum(b - a for a, b in intervals)
    cb = [b for b in bursts if in_climb(b)]
    imu = r['imu']; ts = [x[0] for x in imu]
    halts = []
    for t in tr:
        if t['to'] != 'HALT' or t['frm'] not in ('TRANSITION', 'ASCENT'): continue
        i = bisect.bisect_left(ts, t['t'] - 0.15); j = bisect.bisect_right(ts, t['t'] + 0.05)
        if max(x[3] for x in imu[i:j]) > 0.5: halts.append(t['t'])
    cmd = r['cmd']; ct = [c[0] for c in cmd]
    dip = []; hal = []
    for b in cb:
        i = bisect.bisect_left(ct, b - 0.1); j = bisect.bisect_right(ct, b + 0.6)
        m = min((abs(c[1]) for c in cmd[i:j]), default=None)
        dip.append(m is not None and m < 0.02)
        hal.append(any(b <= h <= b + 0.6 for h in halts))
    n_dip = sum(dip); h_dip = sum(1 for a, b in zip(dip, hal) if a and b); h_nodip = sum(1 for a, b in zip(dip, hal) if (not a) and b)
    per = {'n15a': 3.0, 'n15b': 3.0, 'n30': 6.0, 'n7': 1.4, 'off': 'off'}[arm]
    print(f'{arm:6s} {per!s:>7s} {climb_s:8.0f} {len(cb):16d} {len(halts):20d} {len(halts)/max(1,len(cb)):11.2f} {n_dip:26d} {h_dip:19d} {h_nodip:22d}')
