#!/usr/bin/env python3
"""Independent check on the OLD sharp-world loaded freeze (loaded_payload_diagnosis/ground_truth_pose_trace.csv: chassis frozen at gz x = 7.119 m):
with the ORIGINAL sharp geometry (data/ramp_world_sharp.sdf.ORIGINAL), is the front wheel touching the crest step corner at that chassis position?"""
import math, sys
sys.path.insert(0, '.')
import seam_geometry as sg
P = [p for p in sg.panels('data/ramp_world_sharp.sdf.ORIGINAL') if p['name'] in ('zone_b_ramp', 'zone_c_exit')]
ramp = next(p for p in P if p['name'] == 'zone_b_ramp'); plat = next(p for p in P if p['name'] == 'zone_c_exit')
xa, za, xb, zb = sg.top_edges(ramp); pxa, pza, pxb, pzb = sg.top_edges(plat)
theta = -ramp['pitch']                       # ramp climbs toward +x at 16 deg
print(f"ramp top-face top edge ({xb:.4f}, {zb:.4f}); platform top face starts ({pxa:.4f}, {pza:.4f}) -> step {pza - zb:+.4f} m, x gap {pxa - xb:+.4f}")
x_chassis = 7.1191
R, WX = 0.12, 0.28
fx = x_chassis + WX * math.cos(theta)                      # front wheel centre x (chassis pitched along the slope)
xc = fx + R * math.sin(theta)                              # contact point on the ramp surface
zs = zb - (xb - xc) * math.tan(theta)                      # ramp surface height there
fz = zs + R * math.cos(theta)                              # wheel centre height
corner = (pxa, pza)
d = math.hypot(corner[0] - fx, corner[1] - fz)
print(f"chassis x = {x_chassis}: front wheel centre ({fx:.4f}, {fz:.4f}); distance to the step's top corner {corner} = {d:.4f} m vs wheel radius {R} m  ->  gap {d - R:+.4f} m "
      f"({'TOUCHING the step corner' if d <= R + 0.006 else 'no contact'})")
