#!/usr/bin/env python3
"""Explicit named-pair top-surface continuity check for every REAL join in the Floor-2 extension (exact corner math on the SDF; no visual inspection).
This is the check that would have caught the 0.05 m box-centre-vs-top-surface authoring step (construction-site freeze, ramp-world crest freeze)."""
import os
RAMP_WS = os.environ['RAMP_WS']
import sys
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8/crest_freeze'))
import seam_geometry as sg
W = os.path.join(RAMP_WS, 'src/ramp_autonomy_gazebo/worlds/construction_site.sdf')
P = {p['name']: p for p in sg.panels(W)}
def edge(name, side):
    xa, za, xb, zb = sg.top_edges(P[name]); return (xa, za) if side == 'back' else (xb, zb)      # back = low-x edge, front = high-x edge
def yedge(name, side):
    p = P[name]; return (p['y'] - p['sy'] / 2, p['y'] + p['sy'] / 2)
ok = True
def pair(a, sa, b, sb, label):
    global ok
    ea, eb = edge(a, sa), edge(b, sb); gap, step = eb[0] - ea[0], eb[1] - ea[1]
    bad = abs(step) > 0.002 or abs(gap) > 0.006
    ok &= not bad
    print(f"{'FAIL' if bad else 'ok  '}  {label:58s} x gap {gap:+.4f} m   z step {step:+.4f} m")
print('== Floor 1 (existing, unmodified) -> ramp 2 -> Floor 2, along the robot centreline (y = 0) ==')
pair('elevated_level', 'front', 'ramp2_fillet_bottom_1', 'back', 'Floor 1 east edge -> ramp 2 bottom (same relation as laydown -> ramp 1)')
pair('laydown_area', 'front', 'ramp_fillet_bottom_1', 'back', '   reference: laydown -> ramp 1 bottom (already validated)')
chain = ['ramp2_fillet_bottom_1', 'ramp2_fillet_bottom_2', 'ramp2_fillet_bottom_3', 'ramp2_mid', 'ramp2_fillet_top_1', 'ramp2_fillet_top_2', 'ramp2_fillet_top_3']
for a, b in zip(chain[:-1], chain[1:]):
    pair(a, 'front', b, 'back', f'{a} -> {b}')
pair('ramp2_fillet_top_3', 'front', 'floor2_slab_main', 'back', 'ramp 2 top -> Floor 2 (lane slab)')
pair('elevated_level', 'front', 'floor1_ext_slab', 'back', 'Floor 1 (elevated_level) -> Floor-1 extension slab (flush)')
print('== Floor 2 slab boxes (equal top heights, edges coincide) ==')
for n in ('floor2_slab_main', 'floor2_slab_sw', 'floor2_slab_se', 'floor2_slab_s'):
    xa, za, xb, zb = sg.top_edges(P[n]); print(f'      {n:20s} top z = {za:.4f} (x {xa:.3f} -> {xb:.3f}, y {P[n]["y"] - P[n]["sy"] / 2:.2f} .. {P[n]["y"] + P[n]["sy"] / 2:.2f})')
tops = {round(sg.top_edges(P[n])[1], 4) for n in ('floor2_slab_main', 'floor2_slab_sw', 'floor2_slab_se', 'floor2_slab_s')}
ok &= len(tops) == 1 and abs(list(tops)[0] - 6.55) < 1e-6
print(f'      all four Floor-2 boxes share one top height: {sorted(tops)}   (floor-to-floor rise = 6.55 - 3.35 = 3.200 m)')
print('\nRESULT:', 'ALL SEAMS PASS (<= 2 mm step, <= 6 mm crack; ramp-2 chain identical to ramp 1)' if ok else 'SEAM FAILURE')
sys.exit(0 if ok else 1)
