#!/usr/bin/env python3
"""Standalone, parallel diagnostic observer for the loaded construction-site
freeze. Run alongside (not instead of) the staged round-trip test - immune
to the test script's own timeout-wrapper crash issue since it isn't itself
wrapped in a shell `timeout N` (same pattern as the earlier stall_monitor.py
parallel observer).

Logs, per /joint_states tick:
  - commanded linear.x / angular.z (last /model/ramp_robot/cmd_vel)
  - per-wheel COMMANDED speed (rad/s), derived kinematically from the
    Twist above using the exact same wheel_radius_m=0.12/track_width_m=0.7
    convention as stability_monitor_node.py's own _compute_wheel_slip
  - per-wheel ACTUAL velocity (rad/s) and effort, straight from
    /joint_states (real physics ground truth, independent of any plugin
    doing the actuating)
  - /wheel_slip's own live reading
  - /ramp_state, /motion_stalled

Plus, every ~1s, Gazebo's own ground-truth model pose+orientation
(position AND full quaternion->rpy), polled the same way
arc_turn_followpath_test_ekf.py already does.
"""
import csv
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/wheel_diagnosis.csv'
WORLD_NAME = 'construction_site'
CMD_VEL_TOPIC = '/model/ramp_robot/cmd_vel'
WHEEL_RADIUS_M = 0.12
TRACK_WIDTH_M = 0.7
WHEEL_JOINT_SIDE = {
    'front_left_wheel_joint': 'left',
    'rear_left_wheel_joint': 'left',
    'front_right_wheel_joint': 'right',
    'rear_right_wheel_joint': 'right',
}


def get_gz_pose_full(node_name='ramp_robot'):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=2
        ).stdout
        idx = out.find(f'name: "{node_name}"')
        if idx < 0:
            return None
        block = out[idx:idx + 500]
        pos_i = block.find('position {')
        pos_block = block[pos_i:pos_i + 150]
        ori_i = block.find('orientation {')
        ori_block = block[ori_i:ori_i + 200] if ori_i >= 0 else ''

        def parse_field(txt, key):
            i = txt.find(key + ':')
            if i < 0:
                return None
            j = txt.find('\n', i)
            return float(txt[i + len(key) + 1:j].strip())

        px, py, pz = (parse_field(pos_block, k) for k in ('x', 'y', 'z'))
        qx, qy, qz, qw = (parse_field(ori_block, k) for k in ('x', 'y', 'z', 'w'))
        if None in (qx, qy, qz, qw):
            qx = qy = qz = 0.0
            qw = 1.0
        # roll/pitch/yaw from quaternion (standard convention)
        sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.degrees(math.asin(sinp))
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sinr, cosr))
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(siny, cosy))
        return (px, py, pz, roll, pitch, yaw, qx, qy, qz, qw)
    except Exception as e:
        return None


class WheelDiagnosis(Node):
    def __init__(self):
        super().__init__('wheel_diagnosis')
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.wheel_slip = 0.0
        self.ramp_state = ''
        self.motion_stalled = False
        self.rows = []
        self.t0 = time.time()
        self.last_gz_poll = -999.0
        self.last_gz = None
        self.last_row_print = 0.0

        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 20)
        self.create_subscription(Float32, '/wheel_slip', self._on_slip, 10)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Bool, '/motion_stalled', self._on_stalled, 10)

    def _on_cmd_vel(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def _on_slip(self, msg):
        self.wheel_slip = msg.data

    def _on_state(self, msg):
        self.ramp_state = msg.data

    def _on_stalled(self, msg):
        self.motion_stalled = msg.data

    def _on_joint_states(self, msg: JointState):
        t = time.time() - self.t0
        half_track = TRACK_WIDTH_M / 2.0
        cmd_side = {
            'left': (self.cmd_linear - self.cmd_angular * half_track) / WHEEL_RADIUS_M,
            'right': (self.cmd_linear + self.cmd_angular * half_track) / WHEEL_RADIUS_M,
        }
        wheel_vals = {}
        for name, vel, eff in zip(msg.name, msg.velocity, msg.effort if msg.effort else [None] * len(msg.name)):
            side = WHEEL_JOINT_SIDE.get(name)
            if side is None:
                continue
            wheel_vals[name] = (vel, eff, cmd_side[side])

        now = t
        gz = self.last_gz
        if now - self.last_gz_poll >= 1.0:
            g = get_gz_pose_full()
            self.last_gz_poll = now
            if g:
                gz = g
                self.last_gz = g

        row = {
            't': round(t, 3),
            'cmd_linear': round(self.cmd_linear, 4),
            'cmd_angular': round(self.cmd_angular, 4),
            'wheel_slip': round(self.wheel_slip, 4),
            'ramp_state': self.ramp_state,
            'motion_stalled': self.motion_stalled,
        }
        for wheel in ('front_left_wheel_joint', 'front_right_wheel_joint',
                      'rear_left_wheel_joint', 'rear_right_wheel_joint'):
            vel, eff, cmd = wheel_vals.get(wheel, (None, None, None))
            short = wheel.replace('_wheel_joint', '')
            row[f'{short}_cmd_radps'] = round(cmd, 4) if cmd is not None else ''
            row[f'{short}_actual_radps'] = round(vel, 4) if vel is not None else ''
            row[f'{short}_effort'] = round(eff, 4) if eff is not None else ''
        if gz:
            row['gz_x'], row['gz_y'], row['gz_z'] = (round(v, 4) for v in gz[0:3])
            row['gz_roll'], row['gz_pitch'], row['gz_yaw'] = (round(v, 3) for v in gz[3:6])
        self.rows.append(row)

        if now - self.last_row_print >= 5.0:
            self.last_row_print = now
            self.get_logger().info(
                f't={now:.1f} state={self.ramp_state} stalled={self.motion_stalled} '
                f'cmd=({self.cmd_linear:.3f},{self.cmd_angular:.3f}) slip={self.wheel_slip:.3f} '
                f'gz={gz[0:3] if gz else None} pitch={gz[4] if gz else None}')


def main():
    rclpy.init()
    node = WheelDiagnosis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.rows:
            fieldnames = list(node.rows[0].keys())
            # union of all keys across rows (gz fields only appear once polled)
            for r in node.rows:
                for k in r.keys():
                    if k not in fieldnames:
                        fieldnames.append(k)
            with open(OUT_CSV, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                for r in node.rows:
                    w.writerow(r)
            print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
