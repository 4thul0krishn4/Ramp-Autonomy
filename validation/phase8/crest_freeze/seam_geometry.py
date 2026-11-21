#!/usr/bin/env python3
"""Exact world-frame top-surface geometry of every static box along the y=0 centerline of a world SDF, and the height
STEP / gap at every seam between consecutive panels (precise corner math, no visual inspection).
Usage: seam_geometry.py <world.sdf> [more.sdf ...]"""
import sys, re, math
import xml.etree.ElementTree as ET


def rot_y_pitch(dx, dz, pitch):
    """SDF pose rpy=(0,pitch,0): rotation about +y by pitch. Local (dx, dz) -> world offset (x, z)."""
    return dx * math.cos(pitch) + dz * math.sin(pitch), -dx * math.sin(pitch) + dz * math.cos(pitch)


def panels(path):
    root = ET.parse(path).getroot()
    out = []
    for m in root.iter('model'):
        st = m.find('static')
        pose = m.find('pose')
        if st is None or pose is None:
            continue
        for link in m.findall('link'):
            for col in link.findall('collision'):
                box = col.find('geometry/box/size')
                if box is None:
                    continue
                p = [float(v) for v in pose.text.split()]
                lp = col.find('pose')
                if lp is not None:
                    l = [float(v) for v in lp.text.split()]
                    dx, dz = rot_y_pitch(l[0], l[2], p[4]); p[0] += dx; p[1] += l[1]; p[2] += dz
                sx, sy, sz = [float(v) for v in box.text.split()]
                out.append(dict(name=m.get('name'), x=p[0], y=p[1], z=p[2], roll=p[3], pitch=p[4], yaw=p[5], sx=sx, sy=sy, sz=sz))
    return out


def top_edges(pn):
    """(x_back, z_back, x_front, z_front) of the TOP face centerline: local x = -sx/2 .. +sx/2 at local z = +sz/2."""
    xa, za = rot_y_pitch(-pn['sx'] / 2, pn['sz'] / 2, pn['pitch'])
    xb, zb = rot_y_pitch(+pn['sx'] / 2, pn['sz'] / 2, pn['pitch'])
    return pn['x'] + xa, pn['z'] + za, pn['x'] + xb, pn['z'] + zb


def report(path):
    print(f'\n=== {path.split("/")[-1]} ===')
    P = [p for p in panels(path) if abs(p['y']) <= p['sy'] / 2 and abs(p['roll']) < 1e-6 and abs(p['yaw']) < 1e-6 and p['sy'] > 1.0 and p['sz'] <= 0.12]   # slabs/ramp panels that COVER the y=0 centreline (thin boxes only; walls/columns excluded)
    P.sort(key=lambda p: p['x'])
    print(f"{'panel':28s} {'centre x,z':>14s} {'pitch rad':>9s} {'size x,z':>10s} | top face  x0 -> x1   z0 -> z1")
    E = []
    for p in P:
        xa, za, xb, zb = top_edges(p); E.append((p, xa, za, xb, zb))
        print(f"{p['name']:28s} {p['x']:7.3f},{p['z']:6.3f} {p['pitch']:9.4f} {p['sx']:5.2f},{p['sz']:4.2f} | {xa:7.3f} -> {xb:7.3f}   {za:6.3f} -> {zb:6.3f}")
    print(f"\n{'seam':52s} {'x gap (m)':>10s} {'z STEP (m)':>11s}")
    worst = []
    for (p1, a1, b1, c1, d1), (p2, a2, b2, c2, d2) in zip(E[:-1], E[1:]):
        # order along +x: p1's high-x edge is (c1,d1) (or a1,b1 if pitched the other way) - use whichever edge is nearer to p2
        e1 = min([(a1, b1), (c1, d1)], key=lambda e: abs(e[0] - a2) + abs(e[0] - c2))
        e2 = min([(a2, b2), (c2, d2)], key=lambda e: abs(e[0] - e1[0]))
        gap, step = e2[0] - e1[0], e2[1] - e1[1]
        flag = '   <<<<<< STEP' if abs(step) > 0.01 else ''
        print(f"{p1['name'] + ' -> ' + p2['name']:52s} {gap:+10.4f} {step:+11.4f}{flag}")
        worst.append((abs(step), p1['name'], p2['name'], step, gap))
    return worst


if __name__ == '__main__':
    for w in sys.argv[1:]:
        report(w)
