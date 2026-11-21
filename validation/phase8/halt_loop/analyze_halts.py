#!/usr/bin/env python3
"""Classify every HALT entry of a start_stop_repro.py run: what was the actuation command / wheel state / chassis speed when the slip metric crossed 0.5, i.e. is the 'slip' physical
(wheels spinning while the chassis does not follow) or an artefact of the ratio |cmd - actual| / |cmd| (command small or changing, wheels lagging)?  Usage: analyze_halts.py run.json [...]"""
import bisect, collections, json, statistics as st, sys
R, HALF_TRACK = 0.12, 0.35
def win(seq, t0, t1):
    ts = [x[0] for x in seq]; return seq[bisect.bisect_left(ts, t0):bisect.bisect_right(ts, t1)]
def last_before(seq, t):
    ts = [x[0] for x in seq]; i = bisect.bisect_right(ts, t) - 1; return seq[i] if i >= 0 else None
for f in sys.argv[1:]:
    r = json.load(open(f)); ev = r['ev']; states = ev['state']
    halts = []; prev = 'APPROACH'; last_exit = -9
    for t, s in states:
        if s == 'HALT' and prev != 'HALT': halts.append(dict(t=t, frm=prev, since_exit=t - last_exit))
        if prev == 'HALT' and s != 'HALT': last_exit = t
        prev = s
    print(f'== {f}: legs {len(r["legs"])}, HALT entries {len(halts)}, health {r["health"]}')
    cats = collections.Counter(); rows = []
    for h in halts:
        t = h['t']
        sl = win(ev['slip'], t - 0.25, t + 0.02); slip = max((x[1] for x in sl), default=0.0)
        stall = max((x[1] for x in win(ev['stall'], t - 0.3, t + 0.02)), default=0)
        cmd = last_before(ev['cmd'], t); gat = last_before(ev['gated'], t); jt = last_before(ev['joint'], t); ek = last_before(ev['ekf'], t)
        ek0 = last_before(ev['ekf'], t - 0.6)
        wl = (jt[1] + jt[2]) / 2 if jt else 0.0; wr = (jt[3] + jt[4]) / 2 if jt else 0.0
        cl = ((cmd[1] - cmd[2] * HALF_TRACK) / R) if cmd else 0.0; cr = ((cmd[1] + cmd[2] * HALF_TRACK) / R) if cmd else 0.0
        wheel_v = R * (wl + wr) / 2; chassis_v = ek[1] if ek else 0.0; chassis_v0 = ek0[1] if ek0 else 0.0
        cmd_prev = win(ev['cmd'], t - 0.6, t - 0.3); cmd_prev_v = (sum(x[1] for x in cmd_prev) / len(cmd_prev)) if cmd_prev else 0.0
        recent = h['since_exit'] < 0.6
        onset = abs(chassis_v0) < 0.06 and abs(cmd[1] if cmd else 0) + abs(cmd[2] if cmd else 0) > 0
        decel = cmd and abs(cmd[1]) < abs(cmd_prev_v) - 0.15
        cause = 'stall' if (stall and slip <= 0.5) else 'slip'
        ctx = 'loop (<0.6 s after a HALT exit)' if recent else ('onset from rest' if onset else ('decel/stop' if decel else 'moving'))
        cats[(cause, ctx)] += 1
        rows.append((t, cause, ctx, slip, cmd and cmd[1], cmd and cmd[2], wheel_v, chassis_v, chassis_v0, h['since_exit']))
    for (cause, ctx), n in sorted(cats.items()): print(f'   {cause:5s} {ctx:32s} {n}')
    # physical-slip test: wheels turning much faster than the chassis moves (traction loss) at the HALT instant
    phys = [x for x in rows if x[1] == 'slip' and abs(x[6]) > 0.1 and abs(x[7]) < 0.5 * abs(x[6])]
    print(f'   slip-HALTs where the wheels turn >= 0.1 m/s but the chassis moves < 50 % of wheel speed (physical-looking slip): {len(phys)} of {sum(1 for x in rows if x[1]=="slip")}')
    low = [x for x in rows if x[1] == 'slip' and abs(x[4] or 0) < 0.15 and abs(x[5] or 0) < 0.15]
    print(f'   slip-HALTs with the commanded body twist below 0.15 m/s and 0.15 rad/s (small denominator): {len(low)} of {sum(1 for x in rows if x[1]=="slip")}')
    # loops: >=3 HALT entries with gaps < 1 s
    ts = [h['t'] for h in halts]; loops = []; cur = [ts[0]] if ts else []
    for a, b in zip(ts[:-1], ts[1:]):
        if b - a < 1.0: cur.append(b)
        else:
            if len(cur) >= 3: loops.append(cur)
            cur = [b]
    if len(cur) >= 3: loops.append(cur)
    print(f'   HALT loops (>=3 entries with gaps < 1 s): {len(loops)}, lengths {[len(l) for l in loops]}')
    for x in rows[:12]: print('     t+%.1f %s %-32s slip %.2f cmd(v %.3f, w %.3f) wheel_v %.3f chassis_v %.3f (0.6 s earlier %.3f)' % (x[0] - states[0][0], x[1], x[2], x[3], x[4] or 0, x[5] or 0, x[6], x[7], x[8]))
