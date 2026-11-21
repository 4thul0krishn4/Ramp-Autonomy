#!/usr/bin/env python3
# NOTE (2026-09-19): this script turns the robot with the HISTORICAL open-loop crest turn. On curved worlds that procedure displaces the chassis
# sideways by an amount the EKF cannot see and its curved-world results are UNCONFIRMED - see validation/phase8/path_structural/README.md. Kept unmodified as a record;
# the standard test is validation_test_ekf_groundtruth.py (closed-loop turn on the flat platform for curved worlds).
"""
Cross-track error instrumentation for one failing matrix cell
(curved_unloaded_normal by default). Same ascend -> turn -> FollowPath
descend structure as validation_test_ekf_groundtruth.py, but logs a FULL
timeseries of ground-truth lateral (Y) position throughout the descent
leg (not just the max) - the planned path here is a straight line at
Y=0 (make_path only varies X), so ground-truth |Y| IS the cross-track
error (perpendicular distance from the planned path) directly, no extra
geometry needed. Purpose: determine whether cross-track error grows
UNCHECKED (monotonic, no correction) vs. grows then plateaus/oscillates
(some correction exists but is insufficient).
"""
import math, sys, time, csv, subprocess
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'crosstrack'
WORLD_NAME = sys.argv[2] if len(sys.argv) > 2 else 'ramp_world_curved'
OUT_CSV = sys.argv[3] if len(sys.argv) > 3 else '/tmp/crosstrack.csv'
ASCENT_TIMEOUT_S = 100.0


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


def get_gz_pose(world_name, node_name='ramp_robot'):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world_name}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=2
        ).stdout
        idx = out.find(f'name: "{node_name}"')
        if idx < 0:
            return None
        block = out[idx:idx + 500]
        pos_i = block.find('position {')
        pos_block = block[pos_i:pos_i + 150]

        def parse_field(txt, key):
            i = txt.find(key + ':')
            if i < 0:
                return None
            j = txt.find('\n', i)
            return float(txt[i + len(key) + 1:j].strip())

        return parse_field(pos_block, 'x'), parse_field(pos_block, 'y'), parse_field(pos_block, 'z')
    except Exception:
        return None


class T(Node):
    def __init__(self):
        super().__init__('crosstrack_instrument')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.odom_x = 0.0
        self.odom_yaw = 0.0
        self.reached_descent = False
        self.reached_exit = False
        self.halt_count = 0
        self.t0 = time.time()
        self.rows = []
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        if msg.data == 'HALT' and self.state != 'HALT':
            self.halt_count += 1
        if msg.data == 'DESCENT':
            self.reached_descent = True
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    def spin_for(self, s, period=0.05):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=period)

    def sample(self, tag=''):
        t = time.time() - self.t0
        gz = get_gz_pose(WORLD_NAME)
        row = {'t': round(t, 2), 'tag': tag, 'state': self.state, 'pitch': round(self.pitch, 2),
               'odom_x': round(self.odom_x, 3)}
        if gz:
            row['gz_x'] = round(gz[0], 3)
            row['gz_y_crosstrack'] = round(gz[1], 3)  # path is Y=0, so this IS cross-track error
            row['gz_z'] = round(gz[2], 3)
        self.rows.append(row)
        return row

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

    def send_path_and_wait_instrumented(self, path, timeout_sec, target_x, target_tol=0.5, sample_every=0.3):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        fut = self.client.send_goal_async(goal)
        end = time.time() + 10.0
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{LABEL}] goal rejected')
            return
        rfut = gh.get_result_async()
        deadline = time.time() + timeout_sec
        last_sample = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time() - self.t0
            if now - last_sample >= sample_every:
                last_sample = now
                self.sample('descent')
            if abs(self.odom_x - target_x) < target_tol:
                break
            if rfut.done():
                break
        else:
            gh.cancel_goal_async()
            time.sleep(1.0)
        self.sample('descent_end')


def main():
    rclpy.init()
    node = T()
    node.spin_for(1.0)
    node.client.wait_for_server(timeout_sec=10.0)

    node.get_logger().info(f'[{LABEL}] ascending (budget {ASCENT_TIMEOUT_S:.0f}s)...')
    was_on_ramp = False
    cmd = Twist(); cmd.linear.x = 0.25
    end = time.time() + ASCENT_TIMEOUT_S
    while time.time() < end:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.03)
        if abs(node.pitch) > 10.0:
            was_on_ramp = True
        if was_on_ramp and node.state == 'EXIT':
            node.reached_exit = True
            node.get_logger().info(f'[{LABEL}] reached EXIT')
            break
    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.spin_for(2.0)

    if node.reached_exit:
        node.drive(0.0, 0.5, 6.6)  # turn 180deg
        node.spin_for(1.0)
        node.t0 = time.time()
        yaw_target = math.pi if abs(abs(node.odom_yaw) - math.pi) < 0.5 else 0.0
        path_down = make_path(node.odom_x - 0.5, node.odom_x - 6.5, 0.4, yaw_target,
                               node.get_clock().now().to_msg())
        node.spin_for(0.3)
        node.get_logger().info(f'[{LABEL}] starting instrumented descent (fine-grained cross-track logging)...')
        node.send_path_and_wait_instrumented(path_down, 90.0, node.odom_x - 6.0)

    with open(OUT_CSV, 'w', newline='') as f:
        fieldnames = ['t', 'tag', 'state', 'pitch', 'odom_x', 'gz_x', 'gz_y_crosstrack', 'gz_z']
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in node.rows:
            w.writerow(row)

    descent_rows = [r for r in node.rows if r.get('gz_y_crosstrack') is not None]
    print(f'\n===== CROSS-TRACK RESULT for {LABEL} =====')
    print(f'reached_exit: {node.reached_exit}  reached_descent: {node.reached_descent}  halt_count: {node.halt_count}')
    print(f'n samples: {len(descent_rows)}')
    if descent_rows:
        ys = [r['gz_y_crosstrack'] for r in descent_rows]
        print(f'cross-track error: start={ys[0]:.3f} end={ys[-1]:.3f} max={max(ys, key=abs):.3f}')
        print('timeseries (t, gz_x, cross_track_y):')
        for r in descent_rows:
            print(f"  t={r['t']:6.2f}  gz_x={r['gz_x']:7.3f}  y={r['gz_y_crosstrack']:7.3f}")
    print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
