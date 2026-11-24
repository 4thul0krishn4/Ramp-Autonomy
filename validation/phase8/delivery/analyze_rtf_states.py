#!/usr/bin/env python3
"""Aggregate the repeated real-time RTF runs per payload state. A run is INVALID if at any host sample AC was not online, the power profile was not 'performance', or EPP was not 'performance'.
Per state: per-run mean RTF (5 s windows over 100 s), then across valid runs: mean / min / max of the per-run means, the min and max of ALL 5 s windows, std and 95 % t-interval of the run means, and the
convergence check (running mean of the run means within +-0.005 over the last 3 runs). Usage: analyze_rtf_states.py <dir>"""
import glob, json, statistics as st, sys
if len(sys.argv) < 2:
    raise SystemExit('usage: analyze_rtf_states.py <dir>')
T95 = {2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36, 8: 2.31}
d = sys.argv[1]; by = {}
for f in sorted(glob.glob(f'{d}/*_r[0-9]*.json')):
    if f.endswith('.unloader.log') or 'traj' in f: continue
    hf = f[:-5] + '_health.json'
    guard_starved = False
    try:
        hj = json.load(open(hf)); guard_starved = bool(hj.get('starved'))
    except Exception: pass
    r = json.load(open(f))
    if 'rtf_windows' not in r: continue
    hs = [r['host_start']] + r['host_during'] + [r['host_end']]
    bad = [h for h in hs if str(h['ac']) != '1' or h['profile'] != 'performance' or h['epp'] != 'performance']
    if guard_starved: bad = bad + [dict(reason='guard flagged this run STARVED (NOT COMPLETED per the driver log) - leftover data from a discarded attempt')]
    w = [x[1] for x in r['rtf_windows']]
    by.setdefault(r['state'], []).append(dict(f=f.split('/')[-1], mean=st.mean(w), mn=min(w), mx=max(w), n=len(w), valid=not bad, mhz=st.mean(h['avg_mhz'] for h in r['host_during']), mhz_min=min(h['avg_mhz'] for h in r['host_during']),
                                              unload=r.get('after_unload')))
print(f'{"state":16s} runs valid | mean of run means  min/max of run means | min window .. max window | std   95% CI (t)      | avg CPU MHz during (min run)')
for s in ('unloaded', 'attached', 'attached_plugin', 'detached_idle', 'detached_away'):
    rs = by.get(s, []); ok = [x for x in rs if x['valid']]
    if not ok: print(f'{s:16s} {len(rs)} runs, none valid'); continue
    m = [x['mean'] for x in ok]; ci = (T95.get(len(m) - 1, 2.0) * st.stdev(m) / len(m) ** 0.5) if len(m) > 2 else float('nan')
    conv = 'converged' if len(m) >= 5 and max(abs(st.mean(m[:k]) - st.mean(m)) for k in (len(m) - 2, len(m) - 1)) <= 0.005 else 'NOT converged'
    print(f'{s:16s} {len(rs):3d} {len(ok):5d} | {st.mean(m):.4f}  {min(m):.4f}/{max(m):.4f}      | {min(x["mn"] for x in ok):.3f} .. {max(x["mx"] for x in ok):.3f}    | {st.stdev(m) if len(m) > 1 else 0:.4f} +-{ci:.4f} {conv:13s} | {st.mean(x["mhz"] for x in ok):.0f} ({min(x["mhz_min"] for x in ok):.0f})')
    if len(rs) != len(ok): print(f'      INVALID runs excluded: {[x["f"] for x in rs if not x["valid"]]}')
for s, rs in by.items():
    for x in rs:
        if x['unload'] and 'payload_box' in x['unload']:
            p = x['unload']['payload_box']; pass
print('\nunload check (payload pose after release, detached states):', [(x['f'], x['unload']['payload_box']) for s in ('detached_idle', 'detached_away') for x in by.get(s, []) if x['unload']])
