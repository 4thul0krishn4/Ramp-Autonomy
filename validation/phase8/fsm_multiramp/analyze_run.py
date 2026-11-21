#!/usr/bin/env python3
"""Grade one multiramp_nav2_test.py run: (a) are the /ramp_state labels consistent with the PHYSICAL motion (Gazebo ground-truth dz/dt), (b) does the live controller carry
the parameter profile belonging to the current state (GetParameters on /controller_server), (c) legs / RTF summary.  Usage: analyze_run.py <run.json> [yaml]"""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import bisect, json, sys, yaml, collections
run = json.load(open(sys.argv[1]))
prof = yaml.safe_load(open(sys.argv[2] if len(sys.argv) > 2 else os.path.join(RAMP_WS, 'src/ramp_autonomy_nav/config/ramp_mppi_params.yaml')))
STATE_PROFILE = {'APPROACH': 'flat_mode', 'ASCENT': 'ramp_mode', 'TRANSITION': 'transition_mode', 'DESCENT': 'descent_mode', 'EXIT': 'flat_mode', 'HALT': 'halt_mode'}
gz = run['gz']; gt = [g[0] for g in gz]

def gz_motion(t, w=1.6):
    """physical motion class around time t from ground-truth z: climbing / descending / level (window +-w s, threshold 0.03 m/s)"""
    i0, i1 = bisect.bisect_left(gt, t - w), bisect.bisect_right(gt, t + w) - 1
    if i1 <= i0: return None
    dz = (gz[i1][3] - gz[i0][3]) / max(1e-6, gz[i1][0] - gz[i0][0])
    return 'climbing' if dz > 0.03 else 'descending' if dz < -0.03 else 'level'

print(f'== {sys.argv[1]}: ok={run["ok"]}  legs:')
for l in run['legs']: print(f'   {l["name"]}: {l["why"]}; odom_disp {l["odom_disp"]} GT {l["gt_disp"]} (GT-odom {l["gt_minus_odom"]}) final gz {l["final_gz"]} ok={l["ok"]}')
print('== /ramp_state transitions (pitch: negative = nose up):')
for t in run['transitions']:
    print(f'   t={t["t"]:7.1f} {t["frm"]:>10} -> {t["to"]:<10} pitch={t["pitch"]:7.2f} vx={t["vx"]:6.3f} odom_x={t["odom_x"]:6.2f} gz=({t["gz_x"]}, z={t["gz_z"]}) bp_dist={t["bp_dist"]} conf={t["bp_conf"]}')

print('== labels vs physical motion (ground-truth dz/dt, +-1.6 s window):')
tab = collections.defaultdict(collections.Counter); bad = []
for s in run['samples']:
    m = gz_motion(s['t'])
    if m is None: continue
    tab[s['state']][m] += 1
    if (s['state'] in ('ASCENT', 'TRANSITION') and m == 'descending') or (s['state'] == 'DESCENT' and m == 'climbing'): bad.append((s['t'], s['state'], m, s['odom_x']))
for st in ('APPROACH', 'ASCENT', 'TRANSITION', 'EXIT', 'DESCENT', 'HALT'):
    if st in tab: print(f'   {st:>10}: ' + ', '.join(f'{k} {v}' for k, v in sorted(tab[st].items())) + '   (samples at 2 Hz)')
if bad:
    print(f'   MISLABELLED samples (DESCENT while physically climbing / ASCENT|TRANSITION while descending): {len(bad)}, t {bad[0][0]}..{bad[-1][0]} s, odom_x {bad[0][3]}..{bad[-1][3]}')
else: print('   no mislabelled samples')

print('== live controller parameters vs the profile of the current state (samples where the state has been unchanged >= 2.0 s):')
def match(sv, name):
    p = prof[name]
    return (sv['vx_max'] is not None and abs(sv['vx_max'] - p['max_vel_x']) < 1e-6 and abs(sv['wz_max'] - p['max_vel_theta']) < 1e-6 and abs(sv['align_weight'] - p['path_align_weight']) < 1e-6
            and int(sv['align_offset']) == int(p['path_align_offset']) and abs(sv['angle_tol_rad'] - __import__('math').radians(p['heading_tolerance_deg'])) < 1e-4)
tr = [(t['t'], t['to']) for t in run['transitions']]
def stable_for(t, state):
    since = 0.0
    for tt, to in tr:
        if tt <= t: since = tt
    return t - since
stats = collections.defaultdict(lambda: [0, 0]); ax = collections.defaultdict(list); seen = collections.defaultdict(collections.Counter)
for s in run['samples']:
    if s['vx_max'] is None or stable_for(s['t'], s['state']) < 2.0: continue
    ok = match(s, STATE_PROFILE[s['state']]); stats[s['state']][0] += ok; stats[s['state']][1] += 1
    if s['ax_max'] is not None: ax[s['state']].append(s['ax_max'])
    if not ok: seen[s['state']][(s['vx_max'], s['wz_max'], s['align_weight'], s['align_offset'])] += 1
for st, (a, b) in stats.items():
    axs = ax[st]; print(f'   {st:>10} -> {STATE_PROFILE[st]:<16} {a}/{b} samples carry the expected profile' + (f'; ax_max {min(axs):.2f}..{max(axs):.2f}' if axs else '') + (f'  (non-matching seen: {dict(seen[st])})' if seen[st] else ''))
r = [w[1] for w in run['rtf_windows']]
if r:
    import statistics
    print(f'== sim RTF from /clock (5 s windows, n={len(r)}): mean {sum(r)/len(r):.3f} median {statistics.median(r):.3f} min {min(r):.3f} max {max(r):.3f}')
    per = collections.defaultdict(list)
    for w in run['rtf_windows']: per[w[2]].append(w[1])
    print('   by state at window end: ' + ', '.join(f'{k} {sum(v)/len(v):.3f} (n={len(v)})' for k, v in per.items()))
print(f'== HALT events: {run["halt_count"]}   health: {run.get("health")}')
