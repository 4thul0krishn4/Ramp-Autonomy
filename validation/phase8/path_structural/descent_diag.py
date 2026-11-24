#!/usr/bin/env python3
"""Passive recorder wrapped around the UNMODIFIED validation_test_ekf_groundtruth.py (run as a subprocess).

Records, for the whole cell (ascent, the open-loop 180-degree turn, the descent leg), at native rate:
  * ground truth pose incl. YAW of the chassis (Gazebo, `gz topic --json-output`, independent of every robot sensor)
  * EKF pose/yaw (/odometry/filtered - what controller_server actually tracks) and raw wheel odometry (/odom)
  * the commanded velocity reaching the wheels (/model/ramp_robot/cmd_vel) and the Nav2 command (/cmd_vel_nav)
  * /ramp_state, IMU orientation
and the wall-clock time at which the test logs 'descent path:' (the moment the descent goal is sent).
Usage: descent_diag.py <label> <gz_world_name> <out.npz>
"""
import json, math, os, subprocess, sys, threading, time
RAMP_WS = os.environ['RAMP_WS']
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8/stall_detector_control'))
from stall_control_lib import TruthStream, yaw_from_quat

label, world, out = sys.argv[1], sys.argv[2], sys.argv[3]
HERE = os.path.join(RAMP_WS, 'validation/phase8')


class Rec(Node):
    def __init__(self):
        super().__init__('descent_diag_recorder')
        self.R = {k: [] for k in ('ekf', 'raw', 'act', 'nav', 'imu', 'state')}
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self._odom('ekf', m), 50)
        self.create_subscription(Odometry, '/odom', lambda m: self._odom('raw', m), 50)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', lambda m: self.R['act'].append((time.time(), m.linear.x, m.angular.z)), 100)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.R['nav'].append((time.time(), m.linear.x, m.angular.z)), 100)
        self.create_subscription(Imu, '/imu', self._imu, 100)
        self.create_subscription(String, '/ramp_state', lambda m: self.R['state'].append((time.time(), m.data)), 50)
        self._ic = 0

    def _odom(self, k, m):
        q = m.pose.pose.orientation
        self.R[k].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                          yaw_from_quat(q.x, q.y, q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _imu(self, m):
        self._ic += 1
        if self._ic % 5:
            return
        o = m.orientation
        self.R['imu'].append((time.time(), yaw_from_quat(o.x, o.y, o.z, o.w)))


rclpy.init()
node = Rec()
truth = TruthStream(world=world, min_dt=0.05).start()
env = dict(os.environ)
proc = subprocess.Popen(['timeout', '220', 'python3', f'{HERE}/validation_test_ekf_groundtruth.py', label, world],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, bufsize=1)
lines, t_desc = [], []


def reader():
    for ln in proc.stdout:
        lines.append(ln)
        if 'descent path:' in ln:
            t_desc.append(time.time())


th = threading.Thread(target=reader, daemon=True); th.start()
while proc.poll() is None:
    rclpy.spin_once(node, timeout_sec=0.02)
th.join(timeout=2)
truth.stop()
text = ''.join(lines)
print(text)
R = node.R
np.savez_compressed(out, truth=np.array(truth.samples), ekf=np.array(R['ekf']), raw=np.array(R['raw']),
                    act=np.array(R['act']), nav=np.array(R['nav']), imu=np.array(R['imu']),
                    state_t=np.array([s[0] for s in R['state']]), state=np.array([s[1] for s in R['state']]),
                    t_descent_goal=np.array(t_desc), label=label)
rclpy.shutdown()
