#!/usr/bin/env python3
"""Table of per-run climb metrics for the ramp_mode trigger A/B. Usage: analyze_ab.py <dir> [glob-prefix filter]"""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import bisect, collections, glob, json, re, sys, statistics as st, yaml
d = sys.argv[1]
prof = yaml.safe_load(open(os.path.join(RAMP_WS, 'src/ramp_autonomy_nav/config/ramp_mppi_params.yaml')))
SP = {'APPROACH': 'flat_mode', 'ASCENT': 'ramp_mode', 'TRANSITION': 'transition_mode', 'DESCENT': 'descent_mode', 'EXIT': 'flat_mode', 'HALT': 'halt_mode'}
RAMPS = {1: (28.3, 40.97), 2: (56.3, 68.94)}
import math
def match(s, name):
    p = prof[name]
    return (s['vx_max'] is not None and abs(s['vx_max'] - p['max_vel_x']) < 1e-6 and abs(s['wz_max'] - p['max_vel_theta']) < 1e-6 and abs(s['align_weight'] - p['path_align_weight']) < 1e-6
            and int(s['align_offset']) == int(p['path_align_offset']) and abs(s['angle_tol_rad'] - math.radians(p['heading_tolerance_deg'])) < 1e-4)
rows = []
for f in sorted(glob.glob(f'{d}/*.json')):
    n = f.split('/')[-1][:-5]
    if n.endswith('health'): continue
    if len(sys.argv) > 2 and not n.startswith(sys.argv[2]): continue
    r = json.load(open(f)); gz = r['gz']; imu = r['imu']; ts = [x[0] for x in imu]; tr = r['transitions']
    ok = all(l['ok'] for l in r['legs']) and len(r['legs']) >= 3
    out = dict(name=n, ok=ok)
    for rid, (fx, tx) in RAMPS.items():
        a = next((p[0] for p in gz if p[1] >= fx), None); b = next((p[0] for p in gz if p[1] >= tx and p[3] > (3.3 if rid == 1 else 6.5)), None)
        if a is None or b is None: out[f'r{rid}'] = None; continue
        ys = [abs(p[2]) for p in gz if a <= p[0] <= b]
        halts_slip = halts_stall = 0
        for t in tr:
            if t['to'] == 'HALT' and a <= t['t'] <= b:
                i = bisect.bisect_left(ts, t['t'] - 0.15); j = bisect.bisect_right(ts, t['t'] + 0.05); w = imu[i:j]
                if max(x[3] for x in w) > 0.5: halts_slip += 1
                elif max(x[4] for x in w): halts_stall += 1
        w = [x for x in imu if a <= x[0] <= b]
        # time spent per state during the climb
        tt = collections.Counter(); prev = None
        seq = [(a, None)] + [(t['t'], t['to']) for t in tr if a <= t['t'] <= b]
        st0 = next((t['frm'] for t in tr if a <= t['t'] <= b), None)
        cur = st0 or 'APPROACH'; last = a
        for t in tr:
            if a <= t['t'] <= b:
                tt[cur] += t['t'] - last; last = t['t']; cur = t['to']
        tt[cur] += b - last
        out[f'r{rid}'] = dict(dur=round(b - a, 1), max_y=round(max(ys), 3), h_slip=halts_slip, h_stall=halts_stall, min_margin=round(min(x[2] for x in w), 3), max_slip=round(max(x[3] for x in w), 2),
                              asc=round(tt.get('ASCENT', 0), 1), trn=round(tt.get('TRANSITION', 0), 1), halt_t=round(tt.get('HALT', 0), 1))
    # profile agreement
    cnt = [0, 0]
    for s in r['samples']:
        if s['vx_max'] is None: continue
        since = max([t['t'] for t in tr if t['t'] <= s['t']] or [0.0])
        if s['t'] - since < 2.0: continue
        cnt[1] += 1; cnt[0] += match(s, SP[s['state']])
    out['prof'] = f'{cnt[0]}/{cnt[1]}'
    out['rtf'] = round(st.mean([w[1] for w in r['rtf_windows']]), 3) if r['rtf_windows'] else None
    out['seq'] = ' > '.join(x for i, x in enumerate([tr[0]['frm']] + [t['to'] for t in tr] if tr else []) if x != 'HALT' and (i == 0 or True))
    rows.append(out)
def cell(o, k):
    x = o.get(k); return '-' if x is None else f'{x["dur"]:5.0f}s y{x["max_y"]:.2f} asc{x["asc"]:4.0f} trn{x["trn"]:4.0f} slipH{x["h_slip"]:2d} stallH{x["h_stall"]:2d} m{x["min_margin"]:.2f} s{x["max_slip"]:.1f}'
print(f'{"run":26s} {"ok":3s} | ramp 1 (dur, max|y|, s in ASCENT/TRANSITION, HALTs, min margin, max slip) | ramp 2 | profile-match | RTF')
for o in rows: print(f'{o["name"]:26s} {"Y" if o["ok"] else "N":3s} | {cell(o,"r1")} | {cell(o,"r2")} | {o["prof"]:>8s} | {o["rtf"]}')
