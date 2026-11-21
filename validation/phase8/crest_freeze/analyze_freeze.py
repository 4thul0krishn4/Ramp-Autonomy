#!/usr/bin/env python3
"""Wheel-level analysis of the crest freeze from crest_diag.py recordings + exact contact math against the world's seam geometry.
Usage: analyze_freeze.py <run_crest.npz> [...]"""
import sys, math
import numpy as np
R_W, TRACK, WX, WY = 0.12, 0.70, 0.28, 0.35
STEP_X, STEP_LOW_Z, STEP_HIGH_Z = 8.395, 1.050, 1.100      # zone_c_exit front face; from seam_geometry.py (fillet_top_3 top 1.050, zone_c_exit top 1.100)


def wheel_centres(x, y, z, pitch, yaw):
    """world (x, z) of the four wheel centres in the y=0 plane view: front/rear only (left/right share x,z when roll ~ 0)."""
    c = []
    for dx in (+WX, -WX):
        c.append((x + dx * math.cos(pitch) * math.cos(yaw), z - dx * math.sin(pitch)))
    return c        # [front, rear]


for f in sys.argv[1:]:
    Z = np.load(f, allow_pickle=True); T = Z['truth']; J = Z['joints']; A = Z['act']; S = Z['slip']; E = Z['ekf']; V = Z['verified']; RD = Z['rawdist']; ST = Z['stalled']
    print(f"\n######## {f.split('/')[-1]}")
    if len(T) < 100: print('  no truth'); continue
    t = T[:, 0]
    # freeze = longest window where the chassis moved < 0.02 m over 5 s while the command was forward
    best = None
    for i in range(0, len(T) - 1, 25):
        j = np.searchsorted(t, t[i] + 5.0)
        if j >= len(T): break
        d = math.hypot(T[j, 1] - T[i, 1], T[j, 2] - T[i, 2])
        a = A[(A[:, 0] >= t[i]) & (A[:, 0] <= t[j])]
        if len(a) and np.mean(a[:, 1]) > 0.08 and d < 0.02 and T[i, 1] > 7.5:
            best = (i, j) if best is None else (best[0], j)
    if best is None: print('  no freeze window found (chassis kept moving while commanded forward)'); continue
    i0, j0 = best; t0, t1 = t[i0], t[j0]
    p = T[i0:j0 + 1]; x, y, z, roll, pitch, yaw = p[:, 1].mean(), p[:, 2].mean(), p[:, 3].mean(), p[:, 4].mean(), p[:, 5].mean(), p[:, 6].mean()
    print(f"  FREEZE window {t1 - t0:.1f} s: chassis at gz x={x:.3f} y={y:+.3f} z={z:.3f}, roll {math.degrees(roll):+.1f} pitch {math.degrees(pitch):+.1f} yaw {math.degrees(yaw):+.1f} deg;"
          f" moved {math.hypot(p[-1,1]-p[0,1], p[-1,2]-p[0,2]):.3f} m in the window")
    a = A[(A[:, 0] >= t0) & (A[:, 0] <= t1)]; vx, wz = a[:, 1].mean(), a[:, 2].mean()
    cmd_L, cmd_R = (vx - wz * TRACK / 2) / R_W, (vx + wz * TRACK / 2) / R_W
    jj = J[(J[:, 0] >= t0) & (J[:, 0] <= t1)]
    names = ['front_left', 'rear_left', 'front_right', 'rear_right']
    print(f"  commanded twist (mean): vx={vx:.3f} m/s wz={wz:+.3f} -> commanded wheel speed L={cmd_L:.2f} R={cmd_R:.2f} rad/s")
    for k, nm in enumerate(names):
        act = jj[:, 1 + k].mean(); cmd = cmd_L if 'left' in nm else cmd_R
        print(f"    {nm:12s} commanded {cmd:6.2f} rad/s   ACTUAL {act:6.2f} rad/s   ({100*act/cmd:5.1f}% of command; wheel surface speed {act*R_W:.3f} m/s)")
    print(f"  implied wheel-rotation speed {R_W*np.mean(jj[:,1:].mean(axis=0)):.3f} m/s  vs chassis speed {math.hypot(p[-1,1]-p[0,1], p[-1,2]-p[0,2])/(t1-t0):.4f} m/s")
    s = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    print(f"  /wheel_slip in the window: mean {s[:,1].mean():.3f} max {s[:,1].max():.3f} (HALT threshold 0.5)  n={len(s)}")
    e = E[(E[:, 0] >= t0) & (E[:, 0] <= t1)]
    print(f"  EKF x runs {e[0,1]:.2f} -> {e[-1,1]:.2f} m (+{e[-1,1]-e[0,1]:.2f}) while ground truth x moved {p[-1,1]-p[0,1]:+.3f} m; EKF believes vx={e[:,3].mean():.3f} m/s")
    v0, v1 = np.interp([t0, t1], V[:, 0], V[:, 1]), np.interp([t0, t1], RD[:, 0], RD[:, 1])
    print(f"  /verified_progress_m grew {v0[1]-v0[0]:.2f} m, /raw_wheel_distance_m grew {v1[1]-v1[0]:.2f} m in the window (ground truth moved 0.00 m); stall latched {100*np.mean(np.interp(np.linspace(t0,t1,200), ST[:,0], ST[:,1])):.0f}% of the window")
    (fx, fz), (rx, rz) = wheel_centres(x, y, z, pitch, yaw)
    # exact contact math against the crest -> platform join
    cx, cz = STEP_X, STEP_HIGH_Z
    d = math.hypot(fx - cx, fz - cz)
    print(f"  EXACT GEOMETRY (world x,z): front wheel centre ({fx:.3f}, {fz:.3f}) r={R_W}; rear ({rx:.3f}, {rz:.3f})")
    print(f"     seam: fillet_top_3 top face z={STEP_LOW_Z:.3f} ends at x={STEP_X:.3f}; zone_c_exit top face z={STEP_HIGH_Z:.3f} starts at x=8.395 (vertical face {STEP_HIGH_Z-STEP_LOW_Z:.3f} m high)")
    print(f"     distance front-wheel-centre -> step top corner ({cx:.3f}, {cz:.3f}) = {d:.4f} m  vs wheel radius {R_W} m  =>  {'WHEEL IS TOUCHING/PRESSING THE STEP CORNER' if d <= R_W + 0.006 else 'no contact with the corner'} (gap {d-R_W:+.4f} m)")
    print(f"     wheel-centre height above fillet_top_3 top face: {fz-STEP_LOW_Z:.3f} m (resting radius {R_W}); step corner is {cz-fz:+.3f} m from the centre height")
    print(f"     wheel-corner contact normal angle from vertical: {math.degrees(math.asin(max(-1,min(1,(fz-cz)/R_W)))) if abs(fz-cz)<=R_W else float('nan'):.1f} deg")
