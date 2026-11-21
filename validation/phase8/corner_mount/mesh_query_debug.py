#!/usr/bin/env python3
"""
Debug pass on the Task 6 corner-mount candidate measurements: the first attempt returned the IDENTICAL
surface height (0.12832893927892047) for 4 widely-separated candidate (x,y) points, which is either a real
flat-deck feature of the MiR100 mesh or a bug in how points were threaded into the search. This script
prints the actual mesh-local search window and the raw vertex z-range found for each candidate separately,
so the result can be trusted instead of assumed either way.
"""
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
    print(f'total triangles: {len(tris)}, total vertex-instances: {len(all_verts)}')
    print(f'mesh-local x range: {min(v[0] for v in all_verts):.4f} .. {max(v[0] for v in all_verts):.4f}')
    print(f'mesh-local y range: {min(v[1] for v in all_verts):.4f} .. {max(v[1] for v in all_verts):.4f}')
    print(f'mesh-local z range: {min(v[2] for v in all_verts):.4f} .. {max(v[2] for v in all_verts):.4f}')
    print()

    mir_base_z, mir_base_bottom_gap = 0.3, 0.05
    base_length, base_width, base_height = 0.8, 0.6, 0.25
    mesh_origin_z = -base_height / 2 - mir_base_bottom_gap * (base_height / mir_base_z)
    sx, sy, sz = base_length / 0.9, base_width / 0.58, base_height / mir_base_z
    print(f'scale factors: sx={sx:.4f} sy={sy:.4f} sz={sz:.4f}  mesh_origin_z={mesh_origin_z:.4f}')
    print()

    candidates = {
        'ramp-LiDAR front-left corner (0.35, 0.20)': (0.35, 0.20),
        'ramp-LiDAR front-left, more outboard (0.35, 0.26)': (0.35, 0.26),
        'obstacle-LiDAR rear-right corner (-0.30, -0.20)': (-0.30, -0.20),
        'obstacle-LiDAR rear-right, more outboard (-0.30, -0.26)': (-0.30, -0.26),
        'chassis true corner sanity (0.40, 0.30)': (0.40, 0.30),
    }

    radius = 0.04
    for label, (x_bl, y_bl) in candidates.items():
        xm, ym = x_bl / sx, y_bl / sy
        found = [v for tri in tris for v in tri if abs(v[0] - xm) <= radius and abs(v[1] - ym) <= radius]
        print(f'{label}')
        print(f'  base_link (x,y) = ({x_bl:.3f}, {y_bl:.3f})  ->  mesh-local (xm,ym) = ({xm:.4f}, {ym:.4f})')
        if not found:
            print('  NO VERTICES FOUND in window')
            continue
        zs = [v[2] for v in found]
        xs = [v[0] for v in found]
        ys = [v[1] for v in found]
        print(f'  {len(found)} vertex-instances in window; mesh-local x: {min(xs):.4f}..{max(xs):.4f}, '
              f'y: {min(ys):.4f}..{max(ys):.4f}, z: {min(zs):.4f}..{max(zs):.4f}')
        h = mesh_origin_z + max(zs) * sz
        print(f'  -> surface height rel. base_link = {h:.4f} m (from max z = {max(zs):.4f})')
        print()
