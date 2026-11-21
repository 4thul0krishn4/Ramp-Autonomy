#!/usr/bin/env python3
"""Episodes of the RAW slip metric > THR (default 0.5) in a run recorded with the slip HALT disabled (or any run): duration, peak, and context (onset from rest / decel-stop / turn / moving)
+ whether the wheels were physically slipping (wheel speed vs chassis speed). Usage: slip_episodes.py run.json [thr]"""
import bisect, collections, json, statistics as st, sys
R, HT = 0.12, 0.35
f = sys.argv[1]; THR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
ev = json.load(open(f))['ev']; sl = ev['slip']
def at(seq, t):
    ts = [x[0] for x in seq]; i = bisect.bisect_right(ts, t) - 1; return seq[i] if i >= 0 else None
def win(seq, a, b):
    ts = [x[0] for x in seq]; return seq[bisect.bisect_left(ts, a):bisect.bisect_right(ts, b)]
eps = []; cur = None
for t, v in sl:
    if v > THR:
        if cur is None: cur = [t, t, v]
        else: cur[1] = t; cur[2] = max(cur[2], v)
    elif cur is not None and t - cur[1] > 0.05:
        eps.append(cur); cur = None
if cur: eps.append(cur)
rows = []
for a, b, pk in eps:
    cmd = at(ev['cmd'], a); ek0 = at(ev['ekf'], a - 0.6); ek1 = at(ev['ekf'], a); jt = at(ev['joint'], a)
    cprev = win(ev['cmd'], a - 0.6, a - 0.3); cpv = sum(x[1] for x in cprev) / len(cprev) if cprev else 0.0
    v0 = abs(ek0[1]) if ek0 else 0; w0 = abs(ek0[2]) if ek0 else 0
    onset = v0 < 0.06 and w0 < 0.06
    decel = cmd and abs(cmd[1]) < abs(cpv) - 0.15
    turn = cmd and abs(cmd[1]) < 0.05 and abs(cmd[2]) > 0.05
    ctx = 'onset from rest' if onset else ('decel/stop' if decel else ('turn' if turn else 'moving'))
    wheel_v = R * (jt[1] + jt[2] + jt[3] + jt[4]) / 4 if jt else 0.0; chas = ek1[1] if ek1 else 0.0
    phys = abs(wheel_v) > 0.1 and abs(chas) < 0.5 * abs(wheel_v)
    rows.append((a, b - a, pk, ctx, phys))
print(f'== {f}: raw-slip episodes > {THR}: {len(rows)}')
by = collections.defaultdict(list)
for a, d, pk, ctx, phys in rows: by[ctx].append((d, pk, phys))
for ctx, xs in by.items():
    d = sorted(x[0] for x in xs)
    print(f'   {ctx:16s} n={len(xs):3d}  duration median {st.median(d):.2f} s  p90 {d[int(0.9*(len(d)-1))]:.2f}  max {d[-1]:.2f}   peak median {st.median(x[1] for x in xs):.2f} max {max(x[1] for x in xs):.2f}   physical-looking {sum(x[2] for x in xs)}')
