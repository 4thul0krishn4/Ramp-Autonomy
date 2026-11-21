#!/usr/bin/env python3
"""Passive recorder (no commands): raw /wheel_slip, actuation cmd_vel, wheel joint velocities, EKF twist, /ramp_state, /stability_margin, /motion_stalled with wall-clock stamps, dumped on SIGINT/SIGTERM.
Run alongside any test to capture the slip metric at message rate. Usage: slip_recorder.py <out.json>"""
import json, signal, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
OUT = sys.argv[1]
class R(Node):
    def __init__(self):
        super().__init__('slip_recorder')
        self.ev = dict(state=[], cmd=[], gated=[], joint=[], slip=[], margin=[], stall=[], ekf=[])
        n = time.time
        self.create_subscription(String, '/ramp_state', lambda m: self.ev['state'].append((n(), m.data)), 50)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', lambda m: self.ev['cmd'].append((n(), round(m.linear.x, 4), round(m.angular.z, 4))), 100)
        self.create_subscription(Twist, '/cmd_vel_gated_in', lambda m: self.ev['gated'].append((n(), round(m.linear.x, 4), round(m.angular.z, 4))), 100)
        self.create_subscription(JointState, '/joint_states', self._j, 100)
        self.create_subscription(Float32, '/wheel_slip', lambda m: self.ev['slip'].append((n(), round(m.data, 4))), 100)
        self.create_subscription(Float32, '/stability_margin', lambda m: self.ev['margin'].append((n(), round(m.data, 4))), 50)
        self.create_subscription(Bool, '/motion_stalled', lambda m: self.ev['stall'].append((n(), int(m.data))), 50)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.ev['ekf'].append((n(), round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4), round(m.pose.pose.position.x, 3))), 50)
    def _j(self, m):
        v = dict(zip(m.name, m.velocity))
        self.ev['joint'].append((time.time(), *[round(v.get(k, 0.0), 3) for k in ('front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint')]))
rclpy.init(); node = R(); done = []
def stop(*a): done.append(1)
signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
while not done: rclpy.spin_once(node, timeout_sec=0.1)
json.dump(dict(ev=node.ev, legs=[], health=None), open(OUT, 'w')); print('recorded', {k: len(v) for k, v in node.ev.items()}, flush=True)
