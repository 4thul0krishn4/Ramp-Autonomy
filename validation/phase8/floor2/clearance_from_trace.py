#!/usr/bin/env python3
"""Session-B style clearance check against REAL logged ground-truth points: minimum horizontal distance from every logged ground-truth pose of the
ramp-2 traversal (linearly interpolated between samples) to the footprint of every physical Floor-2 / ramp-2 object. Buffer standard: >= 1.5 m.
Usage: clearance_from_trace.py <ramp2_traversal.csv>"""
import os
RAMP_WS = os.environ['RAMP_WS']
import csv, json, sys
import numpy as np
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r.get('gz_x') not in (None, '')]
P = np.array([[float(r['gz_x']), float(r['gz_y']), float(r['gz_z'])] for r in rows])
# densify (5 cm)
D = [P[0]]
for a, b in zip(P[:-1], P[1:]):
    n = max(1, int(np.hypot(*(b[:2] - a[:2])) / 0.05))
    D += [a + (b - a) * k / n for k in range(1, n + 1)]
D = np.array(D)
L = json.load(open(os.path.join(RAMP_WS, 'validation/phase8/floor2/floor2_layout.json')))
hosts = {'floor1_ext_slab', 'floor2_slab_main', 'floor2_slab_sw', 'floor2_slab_se', 'floor2_slab_s'}
def dist_to_box(pts, b):
    dx = np.maximum(np.maximum(b[0] - pts[:, 0], 0), pts[:, 0] - b[3]); dy = np.maximum(np.maximum(b[1] - pts[:, 1], 0), pts[:, 1] - b[4])
    return np.hypot(dx, dy)
res = []
for m in L['models']:
    if m['name'] in hosts or m['name'].startswith('ramp2_fillet') or m['name'] == 'ramp2_mid' or not m['collision']: continue
    for b in m['boxes']:
        # only compare against objects at (roughly) the same level as the robot points: Floor 2 objects vs points on Floor 2; ramp-2 rails/supports vs points on the ramp/Floor 1
        for lvl, lo, hi in (('floor2', 6.4, 99), ('ramp/floor1', 3.3, 6.4)):
            sel = (D[:, 2] >= lo) & (D[:, 2] < hi)
            if not sel.any(): continue
            if lvl == 'floor2' and b[2] < 6.5: continue
            if lvl == 'ramp/floor1' and (m['group'] not in ('ramp2', 'edge') ): continue
            d = dist_to_box(D[sel][:, :2], b)
            res.append((float(d.min()), m['name'], lvl))
res.sort()
print(f'{len(rows)} logged ground-truth poses ({len(D)} interpolated points); on Floor 2: {(D[:, 2] >= 6.4).sum()} pts; closest objects:')
seen = set()
for d, n, lvl in res:
    if n in seen: continue
    seen.add(n); print(f'   {d:6.2f} m  {n}  ({lvl})')
    if len(seen) >= 12: break
worst = res[0]
print(f'\nMIN CLEARANCE {worst[0]:.2f} m to {worst[1]}  ->', 'PASS (>= 1.5 m buffer)' if worst[0] >= 1.5 else 'BELOW the 1.5 m buffer')
sys.exit(0 if worst[0] >= 1.5 else 1)
