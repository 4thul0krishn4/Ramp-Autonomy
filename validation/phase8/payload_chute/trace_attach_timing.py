#!/usr/bin/env python3
"""Polls the payload_box's gz world pose at high frequency from process start, to see exactly when/how its pose
diverges from the spawn-time design value - specifically testing whether the divergence lines up with the
DetachableJoint actually forming (gz-sim subscribes /payload/state only after the joint exists; this script
polls raw world pose via `gz topic`, which works even before any ROS bridge is up, so it can see the whole
window from before spawn through settling). Run concurrently with the launch (same shell, backgrounded), not after.
Usage: trace_attach_timing.py <out.csv> <duration_s>"""
import subprocess, sys, time, math

OUT, DUR = sys.argv[1], float(sys.argv[2])


def sample():
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'],
                              capture_output=True, text=True, timeout=1.5).stdout
    except Exception:
        return None
    i = out.find('name: "payload_box"')
    if i < 0:
        return None
    blk = out[i:i + 400]
    def getf(b, k):
        a = b.find(k + ':'); c = b.find('\n', a)
        return float(b[a + len(k) + 1:c].strip()) if a >= 0 else None
    jp = blk.find('position {'); posb = blk[jp:jp + 150]
    jo = blk.find('orientation {'); orib = blk[jo:jo + 150]
    px, py, pz = getf(posb, 'x'), getf(posb, 'y'), getf(posb, 'z')
    qx, qw = getf(orib, 'x'), getf(orib, 'w')
    roll = 2 * math.degrees(math.atan2(qx, qw)) if qx is not None else None
    return px, py, pz, roll


t0 = time.time()
rows = ['t,x,y,z,roll_deg,payload_present']
while time.time() - t0 < DUR:
    s = sample()
    t = time.time() - t0
    if s is None:
        rows.append(f'{t:.3f},,,,,0')
    else:
        rows.append(f'{t:.3f},{s[0]:.5f},{s[1]:.5f},{s[2]:.5f},{s[3]:.3f},1')
    time.sleep(0.05)

open(OUT, 'w').write('\n'.join(rows) + '\n')
print(f'wrote {len(rows)-1} samples to {OUT}')
