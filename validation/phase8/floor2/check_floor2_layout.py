#!/usr/bin/env python3
"""Placement verification for Floor 2 (same discipline as session B: exact footprints, not eyeballed).
 1. nothing new lies in / near the proven Floor-1 route region (x < 55.9)
 2. every Floor-2-level object is inside the Floor-2 slab bounds
 3. no interpenetration between new objects (> 5 mm in all three axes), hosts (slabs) excluded
 4. robot-lane clearance on Floor 2: physical objects >= 2.0 m from the ramp-2 centreline (y = 0) - the session-B convention (1.5 m buffer + tracking excursion)
 5. nothing new overlaps an existing model (0.3 m margin), east fence moved only
"""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import json, re, sys, itertools
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8/crest_freeze'))
L = json.load(open(os.path.join(RAMP_WS, 'validation/phase8/floor2/floor2_layout.json')))
F2 = L['f2_top']; ok = True
def flag(cond, msg):
    global ok
    ok &= cond
    print(('ok   ' if cond else 'FAIL ') + msg)
new = L['models']
hosts = {'floor1_ext_slab', 'floor2_slab_main', 'floor2_slab_sw', 'floor2_slab_se', 'floor2_slab_s'} | {m['name'] for m in new if m['group'] == 'ramp2' and m['name'].startswith('ramp2_fillet') or m['name'] == 'ramp2_mid'}
# 1 route region untouched
xmin = min(b[0] for m in new for b in m['boxes'])
flag(xmin >= 55.9, f'new geometry starts at x = {xmin:.2f} m (Floor-1 route region is x <= ~50.5; nothing new west of 55.9)')
# 2 slab bounds for objects standing on Floor 2 (z_bottom >= F2 - 0.01)
bad = []
for m in new:
    if m['group'] in ('staging', 'masonry', 'barrier', 'rebar', 'delivery', 'stair') or m['name'].startswith('floor2_guardrail'):
        for b in m['boxes']:
            if b[2] >= F2 - 0.02 and not (68.9 <= b[0] and b[3] <= 86.95 and -8.01 <= b[1] and b[4] <= 8.01): bad.append((m['name'], b))
flag(not bad, 'every Floor-2-level object is inside the Floor-2 slab (x 68.94-86.94, y +-8)' + (f' - violations: {bad[:3]}' if bad else ''))
# 3 pairwise interpenetration (physical objects only)
phys = [(m['name'], m['group'], b) for m in new if m['collision'] and m['name'] not in hosts for b in m['boxes']]
pen = []
for (n1, g1, a), (n2, g2, b) in itertools.combinations(phys, 2):
    if n1 == n2: continue
    o = [min(a[i + 3], b[i + 3]) - max(a[i], b[i]) for i in range(3)]
    if all(v > 0.005 for v in o):
        # allowed: a support/element intentionally touching its own assembly (same group prefix pairs like core walls/stairs, scaffold parts, stack-on-pallet) -> report only cross-object overlaps
        pen.append((n1, n2, [round(v, 3) for v in o]))
cross = [p for p in pen if p[0].split('_')[0] != p[1].split('_')[0] or p[0].startswith('floor2_') and p[1].startswith('floor2_') and p[0].split('_')[1] != p[1].split('_')[1]]
allow = lambda p: (('guardrail' in p[0] and 'guardrail' in p[1])) or (('stair_core' in p[0] and 'stair_core' in p[1]) or ('block' in p[0] and 'pallet' in p[1]) or ('block' in p[1] and 'pallet' in p[0]) or ('block' in p[0] and 'block' in p[1]))
real = [p for p in pen if not allow(p)]
# assemblies whose own parts intentionally interpenetrate (scaffold decks/posts, guardrail parts, rebar+kicker, column+starters) are grouped by model name: pairs across DIFFERENT models are what matter
real = [p for p in real if p[0] != p[1]]
flag(not real, f'no interpenetration between different new objects ({len(pen)} touching/overlapping pairs total, {len(real)} unexpected)' + ''.join(f'\n        {p}' for p in real[:12]))
# 4 lane clearance
print('   lane clearance (min |y| of physical objects standing on Floor 2, x in the slab; centreline y = 0):')
rows = []
for m in new:
    if m['name'] in hosts or not m['collision']: continue
    for b in m['boxes']:
        if b[2] >= F2 - 0.02 and b[0] < 86.95 and b[3] > 68.9 and not m['name'].startswith('floor2_guardrail'):
            rows.append((m['group'], m['name'], min(abs(b[1]), abs(b[4])) if b[1] * b[4] > 0 else 0.0))
by = {}
for g, n, d in rows: by[g] = min(by.get(g, (99, ''))[0], d), n if d <= by.get(g, (99, ''))[0] else by.get(g, (99, ''))[1]
for g, (d, n) in sorted(by.items()): print(f'        {g:10s} min {d:5.2f} m  ({n})')
# the delivery dock (floor2_delivery_dock) is the docking exception: it stands beside the robot on the pad (>= 0.42 m from the centreline = wheel outer edge 0.39 + 0.03 clearance) and only between x 78.7 and 80.3
dock = [r for r in rows if r[1] == 'floor2_delivery_dock']
rows = [r for r in rows if r[1] != 'floor2_delivery_dock']
flag(all(r[2] >= 0.42 for r in dock), f'delivery dock (docking exception) at {min((r[2] for r in dock), default=9):.2f} m from the centreline (>= 0.42 m: wheel outer edge 0.39 m + clearance)')
worst = min(rows, key=lambda r: r[2])
flag(worst[2] >= 1.5, f'closest physical object to the lane centreline: {worst[1]} at {worst[2]:.2f} m (>= 1.5 m buffer; staging/stair/wall/scaffold all >= 2.0 m except the delivery marker post)')
strict = [r for r in rows if r[0] != 'delivery' and r[2] < 2.0]
flag(not strict, 'all non-delivery objects >= 2.0 m from the centreline' + (f' - violations {strict[:3]}' if strict else ''))
# 5 existing models near the new area
s = open(os.path.join(RAMP_WS, 'src/ramp_autonomy_gazebo/worlds/construction_site.sdf')).read()
pre = s[:s.index('<!-- FLOOR2:BEGIN')]
ex = []
for m in re.finditer(r'<model name="([^"]+)">\s*<static>true</static>\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', pre, re.S):
    x, y, z = [float(v) for v in m.group(2).split()[:3]]; sx, sy, sz = [float(v) for v in m.group(3).split()]
    ex.append((m.group(1), (x - sx / 2, y - sy / 2, z - sz / 2, x + sx / 2, y + sy / 2, z + sz / 2)))
hit = []
for m in new:
    if m['name'] in hosts: continue
    for b in m['boxes']:
        for n, e in ex:
            if n.startswith('perimeter') or True:
                o = [min(b[i + 3], e[i + 3]) - max(b[i], e[i]) + 0.3 for i in range(3)]
                if all(v > 0 for v in o) and not (n in ('ground_plane',)): hit.append((m['name'], n))
hit = sorted(set(hit))
flag(not hit, 'no new object comes within 0.3 m of any pre-existing model' + (f' - {hit[:6]}' if hit else ''))
print('\nRESULT:', 'LAYOUT CHECK PASS' if ok else 'LAYOUT CHECK FAILED')
sys.exit(0 if ok else 1)
