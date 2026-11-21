#!/usr/bin/env python3
"""Lowest-level recorder around the UNMODIFIED validation test (run as a subprocess), for the crest-freeze diagnosis.
Records at native rate: ground-truth pose with FULL orientation (x, y, z, roll, pitch, yaw) from Gazebo; every wheel joint's ACTUAL
angular velocity (/joint_states); the commanded twist reaching the DiffDrive plugin (/model/ramp_robot/cmd_vel), from which the
per-wheel COMMANDED angular velocity follows (differential kinematics: wheel_radius 0.12, track 0.7); /wheel_slip;
/verified_progress_m and /raw_wheel_distance_m and /motion_stalled (the detector's signals); EKF pose; /ramp_state.
Usage: crest_diag.py <label> <gz_world_name> <out.npz>"""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import json, math, os, subprocess, sys, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool, Float32, String

label, world, out = sys.argv[1], sys.argv[2], sys.argv[3]
HERE = os.path.join(RAMP_WS, 'validation/phase8')
WHEELS = ['front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint']


def rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Truth:
    def __init__(self, world, model='ramp_robot', min_dt=0.02):
        self.samples, self._stop, self.model, self.min_dt = [], False, model, min_dt
        self.proc = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '--json-output'],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.th = threading.Thread(target=self._run, daemon=True); self.th.start()

    def _run(self):
        last, needle = 0.0, f'"{self.model}"'
        for line in self.proc.stdout:
            if self._stop: break
            now = time.time()
            if now - last < self.min_dt or needle not in line: continue
            try: msg = json.loads(line)
            except Exception: continue
            for p in msg.get('pose', []):
                if p.get('name') == self.model:
                    pos, o = p.get('position', {}), p.get('orientation', {})
                    r, pt, y = rpy((o.get('x', 0.0), o.get('y', 0.0), o.get('z', 0.0), o.get('w', 1.0)))
                    self.samples.append((now, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), r, pt, y)); last = now; break

    def stop(self):
        self._stop = True
        try: self.proc.kill()
        except Exception: pass


class Rec(Node):
    def __init__(self):
        super().__init__('crest_diag_recorder')
        self.R = {k: [] for k in ('joints', 'act', 'slip', 'verified', 'rawdist', 'stalled', 'ekf', 'state')}
        self.create_subscription(JointState, '/joint_states', self._js, 100)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', lambda m: self.R['act'].append((time.time(), m.linear.x, m.angular.z)), 100)
        self.create_subscription(Float32, '/wheel_slip', lambda m: self.R['slip'].append((time.time(), m.data)), 50)
        self.create_subscription(Float32, '/verified_progress_m', lambda m: self.R['verified'].append((time.time(), m.data)), 50)
        self.create_subscription(Float32, '/raw_wheel_distance_m', lambda m: self.R['rawdist'].append((time.time(), m.data)), 50)
        self.create_subscription(Bool, '/motion_stalled', lambda m: self.R['stalled'].append((time.time(), int(m.data))), 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf, 50)
        self.create_subscription(String, '/ramp_state', lambda m: self.R['state'].append((time.time(), m.data)), 50)

    def _js(self, m):
        v = dict(zip(m.name, m.velocity))
        self.R['joints'].append((time.time(),) + tuple(v.get(j, float('nan')) for j in WHEELS))

    def _ekf(self, m):
        p = m.pose.pose.position
        self.R['ekf'].append((time.time(), p.x, p.y, m.twist.twist.linear.x))


rclpy.init(); node = Rec(); truth = Truth(world)
time.sleep(1.0)
proc = subprocess.Popen(['timeout', '360', 'python3', f'{HERE}/validation_test_ekf_groundtruth.py', label, world],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=dict(os.environ), bufsize=1)
lines = []
th = threading.Thread(target=lambda: [lines.append(l) for l in proc.stdout], daemon=True); th.start()
while proc.poll() is None:
    rclpy.spin_once(node, timeout_sec=0.02)
th.join(timeout=2); truth.stop()
print(''.join(lines))
R = node.R
np.savez_compressed(out, truth=np.array(truth.samples), joints=np.array(R['joints']), act=np.array(R['act']), slip=np.array(R['slip']),
                    verified=np.array(R['verified']), rawdist=np.array(R['rawdist']), stalled=np.array(R['stalled']), ekf=np.array(R['ekf']),
                    state_t=np.array([s[0] for s in R['state']]), state=np.array([s[1] for s in R['state']]), label=label)
rclpy.shutdown()
