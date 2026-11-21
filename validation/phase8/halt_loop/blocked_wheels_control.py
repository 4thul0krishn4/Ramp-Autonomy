#!/usr/bin/env python3
"""Positive control for the persistence-gated slip HALT: a SUSTAINED command-tracking failure. The robot drives (raw cmd_vel_nav, 0.3 m/s) straight into the north perimeter fence on the
laydown and keeps pushing for 12 s: the wheels are blocked, wheel speed collapses while the command stays, so |cmd - actual| / |cmd| stays ~1 for seconds. Records the slip metric, the
state and the time HALT is entered relative to the first sample over 0.5. PRE = HALT within one sample; FIX = HALT after the 0.25 s persistence. Usage: blocked_wheels_control.py <out.json>"""
import json, math, os, sys, time
sys.argv = [sys.argv[0]] + sys.argv[1:]
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool
OUT = sys.argv[1]
class B(Node):
    def __init__(self):
        super().__init__('blocked_wheels_control')
        self.yaw = 0.0; self.y = 0.0; self.ev = dict(slip=[], state=[], stall=[])
        self.create_subscription(Float32, '/wheel_slip', lambda m: self.ev['slip'].append((time.time(), m.data)), 100)
        self.create_subscription(String, '/ramp_state', lambda m: self.ev['state'].append((time.time(), m.data)), 50)
        self.create_subscription(Bool, '/motion_stalled', lambda m: self.ev['stall'].append((time.time(), int(m.data))), 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._e, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
    def _e(self, m):
        q = m.pose.pose.orientation; self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); self.y = m.pose.pose.position.y
    def spin(self, s):
        e = time.time() + s
        while time.time() < e: rclpy.spin_once(self, timeout_sec=0.02)
rclpy.init(); n = B(); n.spin(3.0)
end = time.time() + 40
while time.time() < end:                       # face north (+90 deg)
    err = math.atan2(math.sin(math.pi / 2 - n.yaw), math.cos(math.pi / 2 - n.yaw))
    if abs(err) < math.radians(4): break
    t = Twist(); t.angular.z = math.copysign(min(0.5, max(0.12, 0.8 * abs(err))), err); n.pub.publish(t); rclpy.spin_once(n, timeout_sec=0.03)
for _ in range(10): n.pub.publish(Twist()); n.spin(0.05)
n.spin(1.5)
t_push = time.time(); n.ev['push_start'] = t_push
end = time.time() + 75; blocked_t = None
while time.time() < end:                       # drive north until the fence stops us, then keep pushing 12 s
    t = Twist(); t.linear.x = 0.3; n.pub.publish(t); rclpy.spin_once(n, timeout_sec=0.03)
    if blocked_t is None and n.y > 15.0: blocked_t = time.time()
    if blocked_t is not None and time.time() - blocked_t > 12.0: break
for _ in range(10): n.pub.publish(Twist()); n.spin(0.05)
json.dump(dict(ev=n.ev, blocked_t=blocked_t, final_y=n.y), open(OUT, 'w')); print('final y', n.y, 'blocked_t', blocked_t, flush=True); os._exit(0)
