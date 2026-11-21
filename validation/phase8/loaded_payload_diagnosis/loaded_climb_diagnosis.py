#!/usr/bin/env python3
"""
Root-cause instrumentation for the loaded-payload (100kg, +0.1m fwd CG)
sharp-world climb failure found in phase 8's validation matrix.

Logs, on ONE shared timeline, at a fixed sample rate:
  - /ramp_state           (FSM state - is it even reaching ASCENT?)
  - /stability_margin     (tip-over risk - is HALT's margin check the
                            limiting factor?)
  - /wheel_slip           (traction loss - is the robot spinning wheels
                            without net motion?)
  - IMU pitch             (is the robot physically progressing up the
                            slope at all?)
  - odom.x, odom linear.x (actual achieved forward velocity)
  - commanded /cmd_vel_nav linear.x (what's actually being asked for -
                            constant 0.25 m/s for this whole test, so
                            comparing it against ACHIEVED velocity
                            directly shows whether commands are being
                            honored or the robot is stuck despite them)

Drives a short, fixed ascent attempt (not the full 100s budget) since
the matrix already showed the failure is stable/repeatable well within
that window - this just needs enough samples to see the mechanism.
"""
import math, time, csv, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

DURATION_S = 45.0
SAMPLE_PERIOD_S = 0.2
OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/loaded_climb_diagnosis.csv'


class Diag(Node):
    def __init__(self):
        super().__init__('loaded_climb_diagnosis')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.margin = 1.0
        self.slip = 0.0
        self.odom_x = 0.0
        self.odom_vx = 0.0
        self.cmd_vx = 0.0
        self.rows = []
        self.t0 = time.time()

        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Float32, '/stability_margin', self._on_margin, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_slip, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        self.state = msg.data

    def _on_margin(self, msg):
        self.margin = msg.data

    def _on_slip(self, msg):
        self.slip = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_vx = msg.twist.twist.linear.x

    def sample(self, t):
        self.rows.append({
            't': round(t, 2), 'state': self.state, 'pitch_deg': round(self.pitch, 2),
            'stability_margin': round(self.margin, 4), 'wheel_slip': round(self.slip, 4),
            'odom_x': round(self.odom_x, 3), 'odom_vx': round(self.odom_vx, 3),
            'cmd_vx': round(self.cmd_vx, 3),
        })


def main():
    rclpy.init()
    node = Diag()
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info('driving (constant cmd_vel_nav vx=0.25) while logging full timeline...')
    node.t0 = time.time()
    cmd = Twist(); cmd.linear.x = 0.25
    node.cmd_vx = 0.25
    end = time.time() + DURATION_S
    last_sample = 0.0
    while time.time() < end:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.03)
        now = time.time() - node.t0
        if now - last_sample >= SAMPLE_PERIOD_S:
            last_sample = now
            node.sample(now)

    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    with open(OUT_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t', 'state', 'pitch_deg', 'stability_margin',
                                           'wheel_slip', 'odom_x', 'odom_vx', 'cmd_vx'])
        w.writeheader()
        for row in node.rows:
            w.writerow(row)

    print(f'\nwrote {len(node.rows)} samples to {OUT_CSV}')
    print('\n=== SUMMARY ===')
    print(f'final pitch: {node.pitch:.2f} deg')
    print(f'final state: {node.state}')
    print(f'final odom_x: {node.odom_x:.3f}')
    print(f'max wheel_slip seen: {max(r["wheel_slip"] for r in node.rows):.3f}')
    print(f'min stability_margin seen: {min(r["stability_margin"] for r in node.rows):.3f}')
    print(f'max odom_vx seen: {max(r["odom_vx"] for r in node.rows):.3f} (commanded was 0.25)')
    print(f'mean odom_vx over last 15s: '
          f'{sum(r["odom_vx"] for r in node.rows if r["t"] > DURATION_S-15) / max(1, len([r for r in node.rows if r["t"] > DURATION_S-15])):.3f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
