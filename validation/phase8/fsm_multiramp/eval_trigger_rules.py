#!/usr/bin/env python3
"""Offline evaluation of candidate ASCENT->TRANSITION (crest pre-emption) rules on the recorded perception streams of every MPPI ascent this session (both ramps).
Eligible message = 0 <= breakpoint_distance <= 0.75 and confidence >= 0.5 (the existing rule) [and slope_deg <= -thr (flattening ahead = a crest, not a foot)].
Rule R(k, m): fire when >= k of the last m perception messages were eligible (10 Hz stream). k=1,m=1 without slope = the CURRENT behaviour.
Legit zone per ramp (ground truth): the robot's gz x >= (first top-fillet start - 1.3 m): ramp 1 x >= 37.9, ramp 2 x >= 65.85. A fire before that is a FALSE EARLY fire.
Usage: eval_trigger_rules.py run.json [run.json ...]"""
import bisect, json, sys, collections
FILES = sys.argv[1:]
RAMPS = {1: dict(foot_x=28.3, top_x=40.97, legit_x=37.9), 2: dict(foot_x=56.3, top_x=68.94, legit_x=65.85)}
def load(f):
    r = json.load(open(f)); return r
def climbs(r):
    """[(ramp_id, t_foot_entry, t_top)] from gz log: time gz_x crosses foot_x .. top_x on the way up (ramps in order)"""
    gz = r['gz']; out = []
    for rid, g in RAMPS.items():
        a = next((p[0] for p in gz if p[1] >= g['foot_x'] and p[3] > 0.28 + 0.02 * (rid - 1) + (0 if rid == 1 else 3.2)), None)
        a = next((p[0] for p in gz if p[1] >= g['foot_x']), None)
        b = next((p[0] for p in gz if p[1] >= g['top_x'] and p[3] > (3.3 if rid == 1 else 6.5)), None)
        if a is not None and b is not None: out.append((rid, a, b))
    return out
def gz_x(gz, t):
    ts = [p[0] for p in gz]; i = min(bisect.bisect_left(ts, t), len(gz) - 1); return gz[i][1]
def evaluate(rule, files):
    slope_thr, k, m = rule
    res = []
    for f in files:
        r = load(f)
        if not r.get('percept'): continue
        pc = r['percept']
        for rid, a, b in climbs(r):
            w = [p for p in pc if a - 3 <= p[0] <= b + 2]
            hist = collections.deque(maxlen=m); fire = None
            for p in w:
                el = (0 <= p[1] <= 0.75) and p[3] >= 0.5 and (slope_thr is None or p[2] <= -slope_thr)
                hist.append(el)
                if sum(hist) >= k and len(hist) >= min(m, k): fire = p[0]; break
            gx = None if fire is None else gz_x(r['gz'], fire)
            res.append((f.split('/')[-1][:-5], rid, fire is not None, None if gx is None else gx - RAMPS[rid]['top_x'], None if gx is None else (gx >= RAMPS[rid]['legit_x']), None if fire is None else fire - a))
    return res
rules = [(None, 1, 1), (4, 1, 1), (4, 3, 5), (4, 4, 8), (4, 5, 10), (4, 6, 15), (4, 8, 20), (3, 6, 15), (5, 6, 15), (4, 10, 20)]
print(f'{"rule (slope<=-thr, >=k of last m)":38s} {"climbs":>6s} {"fired":>6s} {"FALSE-EARLY":>12s} {"fire dist. before top (m): min/median/max":>44s}')
for rule in rules:
    res = evaluate(rule, FILES); fired = [x for x in res if x[2]]; early = [x for x in fired if not x[4]]
    d = sorted(-x[3] for x in fired if x[4])
    print(f'  thr={rule[0]!s:4s} k={rule[1]:2d} m={rule[2]:2d} {"(current)" if rule==(None,1,1) else "":9s} {len(res):6d} {len(fired):6d} {len(early):12d}   ' + (f'{d[0]:.2f} / {d[len(d)//2]:.2f} / {d[-1]:.2f}' if d else '-'))
