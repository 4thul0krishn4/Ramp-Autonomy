#!/usr/bin/env python3
"""Combined trace: payload_box world pose (via raw gz topic, works pre-ROS-bridge) AND /payload/state (ROS, confirms
when the DetachableJoint actually forms) on the SAME wall-clock timeline, to find out whether the payload's observed
post-spawn drift happens before or after attachment. Usage: trace_attach_and_state.py <out.csv> <duration_s>"""
import subprocess, sys, time, math, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

OUT, DUR = sys.argv[1], float(sys.argv[2])
t0 = time.time()
state_events = []


class N(Node):
    def __init__(self):
        super().__init__('trace_state')
        self.create_subscription(String, '/payload/state', lambda m: state_events.append((time.time() - t0, m.data)), 10)


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


rclpy.init(); n = N()
spin_thread = threading.Thread(target=lambda: rclpy.spin(n), daemon=True)
spin_thread.start()

rows = ['t,x,y,z,roll_deg,payload_present']
while time.time() - t0 < DUR:
    s = sample()
    t = time.time() - t0
    rows.append(f'{t:.3f},{s[0]:.5f},{s[1]:.5f},{s[2]:.5f},{s[3]:.3f},1' if s else f'{t:.3f},,,,,0')
    time.sleep(0.05)

open(OUT, 'w').write('\n'.join(rows) + '\n')
print(f'wrote {len(rows)-1} pose samples to {OUT}')
print('state events:', state_events)
rclpy.shutdown()
