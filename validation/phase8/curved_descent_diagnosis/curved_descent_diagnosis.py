#!/usr/bin/env python3
# NOTE (2026-09-19): this script turns the robot with the HISTORICAL open-loop crest turn. On curved worlds that procedure displaces the chassis
# sideways by an amount the EKF cannot see and its curved-world results are UNCONFIRMED - see validation/phase8/path_structural/README.md. Kept unmodified as a record;
# the standard test is validation_test_ekf_groundtruth.py (closed-loop turn on the flat platform for curved worlds).
"""Root-cause instrumentation for the curved-world descent chatter /
anomalous-pitch finding (Phase 8 task 3). Ascends normally, turns
around, then descends via FollowPath while logging IMU pitch,
/ramp_state, /wheel_slip, /stability_margin, AND ground-truth Gazebo
world pose (position + full orientation, not just IMU-derived pitch) on
one shared timeline, sampled at a steady rate throughout the descent
leg specifically (where the chatter/anomaly was observed)."""
import math, time, csv, sys, subprocess
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/curved_descent_diag.csv'
WORLD_NAME = sys.argv[2] if len(sys.argv) > 2 else 'ramp_world_curved'


def make_path(x_start, x_end, step, yaw, stamp, frame='odom'):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    x = x_start
    direction = 1.0 if x_end > x_start else -1.0
    while (direction > 0 and x <= x_end) or (direction < 0 and x >= x_end):
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
        x += step * direction
    return path


def get_gz_pose():
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=2
        ).stdout
        idx = out.find('name: "ramp_robot"')
        if idx < 0:
            return None
        block = out[idx:idx + 500]

        def grab(key, start_after):
            i = block.find(start_after)
            if i < 0:
                return None
            i = block.find(key, i)
            j = block.find('\n', i)
            return float(block[i + len(key):j].strip())

        pos_i = block.find('position {')
        pos_block = block[pos_i:pos_i + 150]
        x = grab('x:', pos_block) if False else None
        # simpler: parse sequentially
        def parse_field(txt, key):
            i = txt.find(key + ':')
            if i < 0:
                return None
            j = txt.find('\n', i)
            return float(txt[i + len(key) + 1:j].strip())

        px = parse_field(pos_block, 'x')
        py = parse_field(pos_block, 'y')
        pz = parse_field(pos_block, 'z')
        ori_i = block.find('orientation {')
        ori_block = block[ori_i:ori_i + 200]
        ox = parse_field(ori_block, 'x')
        oy = parse_field(ori_block, 'y')
        oz = parse_field(ori_block, 'z')
        ow = parse_field(ori_block, 'w')
        return (px, py, pz, ox, oy, oz, ow)
    except Exception:
        return None


class Diag(Node):
    def __init__(self):
        super().__init__('curved_descent_diagnosis')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.slip = 0.0
        self.margin = 1.0
        self.odom_x = 0.0
        self.odom_yaw = 0.0
        self.rows = []
        self.t0 = time.time()
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_slip, 10)
        self.create_subscription(Float32, '/stability_margin', self._on_margin, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        self.state = msg.data

    def _on_slip(self, msg):
        self.slip = msg.data

    def _on_margin(self, msg):
        self.margin = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    def spin_for(self, s, period=0.05):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=period)

    def drive(self, vx, wz, s):
        t = Twist(); t.linear.x = vx; t.angular.z = wz
        end = time.time() + s
        while time.time() < end:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.03)
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def sample(self, t, tag):
        pose = get_gz_pose()
        row = {'t': round(t, 2), 'tag': tag, 'state': self.state, 'pitch_imu': round(self.pitch, 2),
               'wheel_slip': round(self.slip, 3), 'stability_margin': round(self.margin, 3),
               'odom_x': round(self.odom_x, 3)}
        if pose:
            px, py, pz, ox, oy, oz, ow = pose
            row.update({'gz_x': round(px, 3), 'gz_y': round(py, 3), 'gz_z': round(pz, 3),
                        'gz_qx': round(ox, 4), 'gz_qy': round(oy, 4), 'gz_qz': round(oz, 4), 'gz_qw': round(ow, 4)})
        self.rows.append(row)


def main():
    rclpy.init()
    node = Diag()
    node.spin_for(1.0)
    node.client.wait_for_server(timeout_sec=10.0)

    node.get_logger().info('ascending...')
    was_on_ramp = False
    cmd = Twist(); cmd.linear.x = 0.2
    end = time.time() + 90.0
    while time.time() < end:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.03)
        if abs(node.pitch) > 10.0:
            was_on_ramp = True
        if was_on_ramp and node.state == 'EXIT':
            node.get_logger().info('reached EXIT')
            break
    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.spin_for(2.0)

    node.drive(0.0, 0.5, 6.6)
    node.spin_for(1.0)

    node.get_logger().info('starting instrumented descent leg...')
    node.t0 = time.time()
    start_x = node.odom_x
    yaw_target = math.pi if abs(abs(node.odom_yaw) - math.pi) < 0.5 else 0.0
    path_down = make_path(node.odom_x - 0.5, node.odom_x - 6.5, 0.4, yaw_target, node.get_clock().now().to_msg())

    goal = FollowPath.Goal()
    goal.path = path_down
    goal.controller_id = 'FollowPath'
    fut = node.client.send_goal_async(goal)
    fend = time.time() + 10.0
    while not fut.done() and time.time() < fend:
        rclpy.spin_once(node, timeout_sec=0.1)
    gh = fut.result()
    if gh is None or not gh.accepted:
        node.get_logger().error('goal rejected')
        rclpy.shutdown()
        return
    rfut = gh.get_result_async()

    deadline = time.time() + 60.0
    last_sample = 0.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time() - node.t0
        if now - last_sample >= 0.8:
            last_sample = now
            node.sample(now, 'descent')
        if abs(node.odom_x - (node.odom_x)) < 0.01 and rfut.done():
            break
        if rfut.done():
            break

    with open(OUT_CSV, 'w', newline='') as f:
        fieldnames = ['t', 'tag', 'state', 'pitch_imu', 'wheel_slip', 'stability_margin',
                      'odom_x', 'gz_x', 'gz_y', 'gz_z', 'gz_qx', 'gz_qy', 'gz_qz', 'gz_qw']
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in node.rows:
            w.writerow(row)
    print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
