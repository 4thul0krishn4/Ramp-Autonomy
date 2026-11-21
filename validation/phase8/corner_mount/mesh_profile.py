#!/usr/bin/env python3
"""Diagnose the self-occlusion regression found in the first corner-mount re-validation run (run_corner_checks.sh):
every break-point measurement collapsed to ~0.04 m regardless of true distance, and the raw forward-cone scan showed
most of it filled with ~0.08-0.10 m self-returns. Hypothesis: the old mount (x=0.35, y=0) sat right at the chassis's
sloped front edge, where the real surface drops away in front of/below the sensor; the new mount (x=0.35, y=0.22)
sits inboard on the wide flat deck (see mesh_query_debug.py), which extends forward underneath the sensor's own
steep-downward rays (FORWARD_CONE_MIN_DEG=-80) and gives them a near-immediate self-return. This profiles real mesh
surface height along the FORWARD direction (increasing x) at several candidate y values, to see whether/where the
deck actually drops away, rather than guessing a fix."""
import struct


def read_binary_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n_tri = struct.unpack('<I', f.read(4))[0]
        tris = []
        for _ in range(n_tri):
            f.read(12)
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            f.read(2)
            tris.append((v1, v2, v3))
    return tris


if __name__ == '__main__':
    tris = read_binary_stl('src/ramp_autonomy_description/meshes/mir100/mir_100_base.stl')
    all_verts = [v for tri in tris for v in tri]

    mir_base_z, mir_base_bottom_gap = 0.3, 0.05
    base_length, base_width, base_height = 0.8, 0.6, 0.25
    mesh_origin_z = -base_height / 2 - mir_base_bottom_gap * (base_height / mir_base_z)
    sx, sy, sz = base_length / 0.9, base_width / 0.58, base_height / mir_base_z

    radius = 0.03
    for y_bl in (0.0, 0.10, 0.18, 0.22, 0.26, 0.29):
        print(f'y_bl={y_bl:.2f}:')
        for x_bl in (0.30, 0.33, 0.35, 0.37, 0.39, 0.41, 0.43, 0.45):
            xm, ym = x_bl / sx, y_bl / sy
            zs = [v[2] for v in all_verts if abs(v[0] - xm) <= radius and abs(v[1] - ym) <= radius]
            if not zs:
                print(f'  x_bl={x_bl:.2f}: NO VERTICES (past mesh extent)')
                continue
            h = mesh_origin_z + max(zs) * sz
            print(f'  x_bl={x_bl:.2f}: surface={h:.4f} m rel. base_link ({len(zs)} pts)')
