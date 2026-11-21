#!/usr/bin/env python3
"""Traces BOTH ramp_robot and payload_box world pose/orientation on the same timeline, to test whether the payload's
apparent post-spawn tipping is really sliding relative to the chassis, or just the whole rigidly-joined assembly
rocking/settling together (e.g. as Nav2's controller activates and the drive wheels switch from passive to actively
held). Usage: trace_robot_and_payload.py <out.csv> <duration_s>"""
import subprocess, sys, time, math

OUT, DUR = sys.argv[1], float(sys.argv[2])


def sample():
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'],
                              capture_output=True, text=True, timeout=1.5).stdout
    except Exception:
        return None, None
    def block(name):
        i = out.find(f'name: "{name}"')
        return out[i:i + 400] if i >= 0 else None
    def getf(b, k):
        a = b.find(k + ':'); c = b.find('\n', a)
        return float(b[a + len(k) + 1:c].strip()) if (b and a >= 0) else None
    def pose_of(blk):
        if blk is None:
            return None
        jp = blk.find('position {'); posb = blk[jp:jp + 150]
        jo = blk.find('orientation {'); orib = blk[jo:jo + 150]
        px, py, pz = getf(posb, 'x'), getf(posb, 'y'), getf(posb, 'z')
        qx, qw = getf(orib, 'x'), getf(orib, 'w')
        roll = 2 * math.degrees(math.atan2(qx, qw)) if qx is not None else None
        return px, py, pz, roll
    return pose_of(block('payload_box')), pose_of(block('ramp_robot'))


t0 = time.time()
rows = ['t,pay_x,pay_y,pay_z,pay_roll,rob_x,rob_y,rob_z,rob_roll']
while time.time() - t0 < DUR:
    p, r = sample()
    t = time.time() - t0
    pv = ','.join(f'{x:.5f}' if x is not None else '' for x in p) if p else ',,,'
    rv = ','.join(f'{x:.5f}' if x is not None else '' for x in r) if r else ',,,'
    rows.append(f'{t:.3f},{pv},{rv}')
    time.sleep(0.05)

open(OUT, 'w').write('\n'.join(rows) + '\n')
print(f'wrote {len(rows)-1} samples to {OUT}')
