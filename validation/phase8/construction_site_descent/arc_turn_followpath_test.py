#!/usr/bin/env python3
"""Full construction-site traversal test, v6.

v2-v5 all built the turnaround out of an open-loop, dead-reckoned in-place
spin (command a fixed angular velocity for a fixed duration, then trust
whatever heading source - wheel odometry or IMU - after the fact). Both
sensors disagreed with each other AND with ground truth after that spin
(odom read -175.9 deg, IMU read 135.8 deg, true drift implied ~35-40+ deg
error) - an in-place spin demands maximum rotational force against static
friction with zero forward velocity, close to worst-case for inducing
wheel slip, and unlike every other maneuver in this project it never gets
any live feedback correction while it's happening. No amount of choosing
a different sensor to trust AFTERWARD fixes an open-loop maneuver.

v6 removes the open-loop spin entirely. The turnaround is now part of a
SINGLE continuous Nav2 FollowPath goal - a closed-loop, continuously-
corrected path that arcs the robot 180 degrees around using the elevated
level's own open space (15 x 12 m, confirmed in construction_site.sdf:
pose x=48.47 y=0, size 15x12 -> spans X:[40.97,55.97] Y:[-6,6], comfortably
larger than the R=2.0 m arc used here), tapers back onto the ramp's Y=0
centerline, and continues straight across the ramp itself - the same
MPPI controller that already produced a clean, zero-lateral-drift ascent
handles the whole thing under continuous correction, exactly like every
other maneuver in this project that's held up under testing. Only the
final flat stretch (laydown + access road back to the gate, well after
heading is already correctly established) uses raw cmd_vel, matching the
ascent's own already-proven-driftless flat-ground behaviour.
"""
import math, time, csv, sys, subprocess
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/site_traversal7.csv'
WORLD_NAME = 'construction_site'
DRIVE_VX = 0.4
GATE_X = 1.0
LAYDOWN_END_X = 28.0
RAMP_TOP_X = 40.9702
RAMP_BOTTOM_X = 28.2987
ELEV_MID_X = RAMP_TOP_X + 6.0        # 46.9702, where leg 2 ends
RAMP_APPROACH_X = RAMP_TOP_X + 0.5   # 41.4702
RAMP_EXIT_X = RAMP_BOTTOM_X - 2.0    # 26.2987

ARC_RADIUS = 2.0   # apex bulge = 2.0 m fwd, end offset = 4.0 m in Y - comfortable inside the 6 m platform half-width
ABORT_MARGIN = 0.15
ABORT_PITCH_DEG = 45.0


def quat_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (qz, qw)


def build_turn_and_ramp_path(start_x, start_y, radius, ramp_approach_x, ramp_exit_x, stamp, frame='odom', step=0.4):
    """One continuous path: semicircular arc (180 deg turn, using open
    elevated-level space) -> linear taper back to the ramp's Y=0
    centreline -> straight run across the ramp itself. All in one Path
    message so Nav2's FollowPath/MPPI controller corrects it continuously
    from the very first waypoint - no separate open-loop manoeuvre first."""
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    pts = []  # (x, y, yaw)

    # Phase 1: semicircle, radius R, turning toward +Y. phi: 0 -> pi.
    dphi = step / radius
    n = max(1, int(round(math.pi / dphi)))
    for i in range(n + 1):
        phi = math.pi * i / n
        x = start_x + radius * math.sin(phi)
        y = start_y + radius * (1.0 - math.cos(phi))
        pts.append((x, y, phi))
    arc_end_x, arc_end_y = pts[-1][0], pts[-1][1]

    # Phase 2: taper from the arc's end back onto the ramp centreline
    # (y=0) while continuing toward the ramp, heading held at pi.
    taper_dist = math.hypot(ramp_approach_x - arc_end_x, 0.0 - arc_end_y)
    n2 = max(1, int(round(taper_dist / step)))
    for i in range(1, n2 + 1):
        f = i / n2
        x = arc_end_x + f * (ramp_approach_x - arc_end_x)
        y = arc_end_y + f * (0.0 - arc_end_y)
        pts.append((x, y, math.pi))

    # Phase 3: straight across the ramp itself, y=0, heading pi.
    ramp_dist = abs(ramp_exit_x - ramp_approach_x)
    n3 = max(1, int(round(ramp_dist / step)))
    for i in range(1, n3 + 1):
        f = i / n3
        x = ramp_approach_x + f * (ramp_exit_x - ramp_approach_x)
        pts.append((x, 0.0, math.pi))

    for (x, y, yaw) in pts:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        qz, qw = quat_from_yaw(yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
    return path


def get_gz_pose(node_name='ramp_robot'):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=2
        ).stdout
        idx = out.find(f'name: "{node_name}"')
        if idx < 0:
            return None
        block = out[idx:idx + 400]
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


class Test(Node):
    def __init__(self):
        super().__init__('site_traversal_test6')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.margin = 1.0
        self.slip = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.rows = []
        self.halt_count = 0
        self.min_margin = 1.0
        self.max_slip = 0.0
        self.aborted = False
        # Descent-leg-specific stats (from the arc-turn goal onward), reported
        # separately per the user's request - not just overall/ascent numbers.
        self.descent_active = False
        self.descent_min_margin = 1.0
        self.descent_max_slip = 0.0
        self.descent_max_abs_pitch = 0.0
        self.descent_max_abs_gz_y = 0.0
        self.descent_halt_count = 0
        self.t0 = time.time()
        self.last_gz_poll = -999.0
        self.last_gz = None
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Float32, '/stability_margin', self._on_margin, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_slip, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.follow_client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp))
        if self.descent_active:
            self.descent_max_abs_pitch = max(self.descent_max_abs_pitch, abs(self.pitch))

    def _on_state(self, msg):
        if msg.data != self.state and msg.data == 'HALT':
            self.halt_count += 1
            if self.descent_active:
                self.descent_halt_count += 1
        self.state = msg.data

    def _on_margin(self, msg):
        self.margin = msg.data
        self.min_margin = min(self.min_margin, msg.data)
        if self.descent_active:
            self.descent_min_margin = min(self.descent_min_margin, msg.data)

    def _on_slip(self, msg):
        self.slip = msg.data
        self.max_slip = max(self.max_slip, msg.data)
        if self.descent_active:
            self.descent_max_slip = max(self.descent_max_slip, msg.data)

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def hard_stop(self, n=15):
        stop = Twist()
        for _ in range(n):
            try:
                self.cmd_pub.publish(stop)
            except Exception as e:
                self.get_logger().warn(f'hard_stop publish failed: {e}')
                break
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                pass

    def sample(self, tag=''):
        t = time.time() - self.t0
        if t - self.last_gz_poll >= 1.5 or tag:
            gz = get_gz_pose()
            self.last_gz_poll = t
            if gz:
                self.last_gz = gz
                if self.descent_active:
                    self.descent_max_abs_gz_y = max(self.descent_max_abs_gz_y, abs(gz[1]))
        gz = self.last_gz
        row = {'t': round(t, 2), 'tag': tag, 'state': self.state, 'pitch': round(self.pitch, 2),
               'margin': round(self.margin, 3), 'slip': round(self.slip, 3),
               'odom_x': round(self.odom_x, 3), 'odom_y': round(self.odom_y, 3)}
        if gz:
            row['gz_x'], row['gz_y'], row['gz_z'] = (round(v, 3) for v in gz)
        self.rows.append(row)

    def drive_to(self, target_x, vx, sample_every=2.0, timeout=400.0):
        last_sample = 0.0
        cmd = Twist()
        cmd.linear.x = vx
        deadline = time.time() + timeout
        going_forward = target_x >= self.odom_x
        try:
            while time.time() < deadline:
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time() - self.t0
                if now - last_sample >= sample_every:
                    last_sample = now
                    self.sample()
                if going_forward and self.odom_x >= target_x:
                    break
                if not going_forward and self.odom_x <= target_x:
                    break
        finally:
            self.hard_stop()
        self.sample(tag=f'reached_x={target_x}')

    def follow_arc_and_ramp(self, sample_every=0.5, timeout=180.0):
        self.follow_client.wait_for_server(timeout_sec=10.0)
        start_x, start_y = self.odom_x, self.odom_y
        path = build_turn_and_ramp_path(
            start_x, start_y, ARC_RADIUS, RAMP_APPROACH_X, RAMP_EXIT_X,
            self.get_clock().now().to_msg())
        self.get_logger().info(
            f'FollowPath arc+ramp: {len(path.poses)} waypoints, start=({start_x:.2f},{start_y:.2f}), '
            f'arc radius={ARC_RADIUS}, target_x={RAMP_EXIT_X:.2f}')

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        fut = self.follow_client.send_goal_async(goal)
        fend = time.time() + 10.0
        while not fut.done() and time.time() < fend:
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('FollowPath goal rejected')
            return False
        rfut = gh.get_result_async()

        self.descent_active = True
        last_sample = 0.0
        deadline = time.time() + timeout
        aborted_here = False
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.time() - self.t0
                if now - last_sample >= sample_every:
                    last_sample = now
                    self.sample()
                if self.margin < ABORT_MARGIN or abs(self.pitch) > ABORT_PITCH_DEG:
                    self.get_logger().error(
                        f'SAFETY ABORT: margin={self.margin:.3f} pitch={self.pitch:.1f} - cancelling FollowPath goal')
                    aborted_here = True
                    self.aborted = True
                    try:
                        gh.cancel_goal_async()
                    except Exception:
                        pass
                    break
                if rfut.done():
                    break
        finally:
            self.hard_stop()
        self.sample(tag='followpath_arc_ramp_done' + ('_ABORTED' if aborted_here else ''))
        return not aborted_here


def main():
    rclpy.init()
    node = Test()
    try:
        end = time.time() + 1.5
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.sample(tag='start')

        node.get_logger().info('leg 1: gate -> laydown area...')
        node.drive_to(LAYDOWN_END_X, DRIVE_VX)
        node.get_logger().info(f'leg 1 done: odom_x={node.odom_x:.2f} state={node.state} margin={node.margin:.3f}')

        node.get_logger().info('leg 2: laydown -> up the ramp -> onto elevated level...')
        node.drive_to(ELEV_MID_X, DRIVE_VX, sample_every=1.0)
        node.get_logger().info(f'leg 2 done: odom_x={node.odom_x:.2f} pitch={node.pitch:.2f} state={node.state}')

        node.get_logger().info('leg 3: closed-loop FollowPath arc-turn (elevated level) -> taper onto ramp centreline -> across the ramp...')
        ok = node.follow_arc_and_ramp(sample_every=0.5)
        node.get_logger().info(
            f'leg 3 done: ok={ok} odom_x={node.odom_x:.2f} odom_y={node.odom_y:.2f} pitch={node.pitch:.2f} state={node.state}')

        if not node.aborted:
            node.get_logger().info('leg 4: laydown + access road back to the gate (raw cmd_vel, heading already established)...')
            node.drive_to(GATE_X, DRIVE_VX)
            node.get_logger().info(f'leg 4 done: odom_x={node.odom_x:.2f} state={node.state}')
    finally:
        node.hard_stop()
        node.descent_active = False

    with open(OUT_CSV, 'w', newline='') as f:
        fieldnames = ['t', 'tag', 'state', 'pitch', 'margin', 'slip', 'odom_x', 'odom_y', 'gz_x', 'gz_y', 'gz_z']
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in node.rows:
            w.writerow(row)

    print(f'\n=== TRAVERSAL RESULT ===')
    print(f'aborted: {node.aborted}')
    print(f'total HALT events: {node.halt_count}')
    print(f'min stability_margin observed (overall): {node.min_margin:.3f}')
    print(f'max wheel_slip observed (overall): {node.max_slip:.3f}')
    print(f'--- descent leg (arc-turn + ramp crossing) specific ---')
    print(f'descent HALT events: {node.descent_halt_count}')
    print(f'descent min stability_margin: {node.descent_min_margin:.3f}')
    print(f'descent max wheel_slip: {node.descent_max_slip:.3f}')
    print(f'descent max |pitch|: {node.descent_max_abs_pitch:.2f} deg')
    print(f'descent max |ground-truth lateral Y|: {node.descent_max_abs_gz_y:.3f} m')
    print(f'final gz pose: {node.last_gz}')
    print(f'final odom: x={node.odom_x:.2f} y={node.odom_y:.2f}')
    print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
