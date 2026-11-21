#!/usr/bin/env python3
"""Re-measurement of ramp_classifier_node's break-point accuracy (README "Measured accuracy", phase 4) against the CORRECTED (raised-ramp) geometry.
Same method as the original: robot driven forward, stopped at fixed positions; the perception's breakpoint_distance_m compared with ground truth =
marker_x - (robot_x + 0.35 m LiDAR forward mount offset), markers at x = 4.0 (bottom break point) and x = 7.5 (top). Ground truth here is Gazebo's model pose
(the original used /odom). Usage: breakpoint_accuracy.py <sharp|curved> <out.json>"""
import json, math, statistics, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ramp_autonomy_msgs.msg import RampPercept

KIND, OUT = sys.argv[1], sys.argv[2]
WORLD = 'ramp_world_sharp' if KIND == 'sharp' else 'ramp_world_curved'
MOUNT = 0.35
# (label, marker x, target chassis x)
STOPS = {
    'sharp': [('Zone A, bottom break point ahead (spawn)', 4.0, 1.0), ('Zone A, close to bottom break point', 4.0, 3.34),
              ('On ramp, top break point ahead, far', 7.5, 5.0), ('On ramp, top break point ahead, closer', 7.5, 6.0), ('On ramp, top break point ahead, imminent', 7.5, 6.55)],
    'curved': [('Zone A, fillet start ahead (spawn)', 4.0, 1.0), ('Zone A, close to fillet start (~1.42 m)', 4.0, 2.23)],
}[KIND]


def gz_x():
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=3).stdout
        i = out.find('name: "ramp_robot"'); blk = out[i:i + 500]; j = blk.find('position {'); pb = blk[j:j + 150]
        k = pb.find('x:'); return float(pb[k + 2:pb.find('\n', k)])
    except Exception:
        return None


class N(Node):
    def __init__(self):
        super().__init__('breakpoint_accuracy')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.samples = []      # (t, dist, slope, conf)
        self.state, self.trans = 'APPROACH', []
        self.last = None
        self.create_subscription(RampPercept, '/ramp_perception', self._p, 10)
        self.create_subscription(String, '/ramp_state', self._s, 10)

    def _p(self, m):
        self.last = (m.breakpoint_distance_m, m.slope_deg, m.confidence)
        self.samples.append((time.time(), m.breakpoint_distance_m, m.slope_deg, m.confidence))

    def _s(self, m):
        if m.data != self.state:
            self.trans.append(dict(t=time.time(), frm=self.state, to=m.data, percept_at_transition=self.last)); self.state = m.data

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_to(self, target, vx=0.25, timeout=90):
        end = time.time() + timeout; last_poll = 0.0; t = Twist()
        while time.time() < end:
            x = gz_x() if time.time() - last_poll > 0.6 else None
            if x is not None:
                last_poll = time.time()
                if x >= target - 0.02: break
                t.linear.x = vx if target - x > 0.5 else 0.12
            self.pub.publish(t if t.linear.x else Twist()); rclpy.spin_once(self, timeout_sec=0.03)
        for _ in range(10): self.pub.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.05)


rclpy.init(); n = N(); n.spin(2.0)
rows = []
for label, marker, target in STOPS:
    if target > 1.05: n.drive_to(target)
    n.spin(3.0)                                # settle
    t0 = time.time(); n.spin(3.0)              # measurement window (stationary)
    x = gz_x()
    w = [s for s in n.samples if s[0] >= t0]
    valid = [s for s in w if s[1] >= 0]
    true = marker - (x + MOUNT) if x is not None else float('nan')
    r = dict(stop=label, marker_x=marker, chassis_x=x, true_distance_m=round(true, 3), n=len(w), n_valid=len(valid))
    if valid:
        r.update(measured_m=round(statistics.median(s[1] for s in valid), 3), error_m=round(statistics.median(s[1] for s in valid) - true, 3),
                 slope_deg=round(statistics.median(s[2] for s in valid), 1), confidence=round(statistics.median(s[3] for s in valid), 2))
    rows.append(r); print(r, flush=True)
# continue up to the platform to see what triggers ASCENT -> TRANSITION (sharp only; robot is on the ramp)
if KIND == 'sharp':
    n.drive_to(9.0, vx=0.25, timeout=60); n.spin(2.0)
res = dict(kind=KIND, rows=rows, transitions=[dict(frm=t['frm'], to=t['to'], percept_at_transition=t['percept_at_transition']) for t in n.trans])
json.dump(res, open(OUT, 'w'), indent=1); print('transitions:', res['transitions'])
rclpy.shutdown()
