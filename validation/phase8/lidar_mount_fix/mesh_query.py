#!/usr/bin/env python3
"""
One-off diagnostic for the LiDAR-floating bug (see README.md section 2): measures the real MiR100 chassis mesh's own surface height, in the
base_link frame, directly underneath each sensor's mount x/y - the number the original lidar_joint/obstacle_lidar_joint z-offsets should have been
computed from, instead of the old placeholder box's base_height/2. Reads the STL directly (binary format, struct-parsed) rather than assuming a
mesh library is installed. Not part of any launch/build - a measurement script, kept for anyone re-deriving these numbers after a future mesh change.
"""
import struct


def read_binary_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n_tri = struct.unpack('<I', f.read(4))[0]
        tris = []
        for _ in range(n_tri):
            f.read(12)  # normal
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            f.read(2)  # attribute byte count
            tris.append((v1, v2, v3))
    return tris


def surface_height_rel_base_link(tris, x_target, y_target, radius, mesh_origin_z, sz):
    """Max mesh-local z (the top surface) among vertices within `radius` of (x_target, y_target) in the mesh's own local
    frame, converted to base_link-relative height via the same scale/offset robot.urdf.xacro applies to the chassis visual."""
    zs = [v[2] for tri in tris for v in tri if abs(v[0] - x_target) <= radius and abs(v[1] - y_target) <= radius]
    if not zs:
        return None, 0
    return mesh_origin_z + max(zs) * sz, len(zs)


if __name__ == '__main__':
    tris = read_binary_stl('src/ramp_autonomy_description/meshes/mir100/mir_100_base.stl')

    # Same mir_base_* / scale constants robot.urdf.xacro uses for the chassis visual.
    mir_base_z, mir_base_bottom_gap = 0.3, 0.05
    base_length, base_width, base_height = 0.8, 0.6, 0.25
    mesh_origin_z = -base_height / 2 - mir_base_bottom_gap * (base_height / mir_base_z)
    sx, sy, sz = base_length / 0.9, base_width / 0.58, base_height / mir_base_z

    lidar_x_mesh_local = (base_length / 2 - 0.05) / sx
    h, n = surface_height_rel_base_link(tris, lidar_x_mesh_local, 0.0, 0.04, mesh_origin_z, sz)
    print(f'lidar_link mount (mesh-local x={lidar_x_mesh_local:.4f}): real surface = {h:.4f} m rel. base_link ({n} vertices)')

    h, n = surface_height_rel_base_link(tris, 0.0, 0.0, 0.04, mesh_origin_z, sz)
    print(f'obstacle_lidar_link mount (mesh-local x=0,y=0): real surface = {h:.4f} m rel. base_link ({n} vertices)')
