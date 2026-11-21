#!/usr/bin/env python3
"""Diagnostic rerun of leg 3 (arc-turn + taper + ramp) + leg 4 (open-loop
return), instrumenting an external, code-consistent proxy for
PathAlignCritic's own activation logic (there is no live ROS topic for
per-critic cost - nav2_mppi_controller's `visualize` output is trajectory
markers only, not cost breakdowns - so this reconstructs the SAME check
path_align_critic.cpp itself performs, from data available outside the
controller: the exact Path message sent, /ramp_state (which selects the
profile and therefore offset_from_furthest), and the robot's live position).

path_align_critic.cpp's own activation gate, reproduced exactly:
    path_segments_count = *data.furthest_reached_path_point   # monotonic,
                                                                # index into
                                                                # the sent Path
    if path_segments_count < offset_from_furthest_: return    # DISABLED

`furthest_reached_path_point` is nav2_mppi_controller's own internal
path-handler state (tools/path_handler.cpp) - not published anywhere -
approximated here as a running max of "index of the path point nearest
the robot's current position," which is what that internal tracker is
built from (monotonic progress along the path). This is a proxy, not the
literal internal value - treated as such throughout, and cross-checked
against the actual cross-track deviation trend (if the proxy says
"critic active" but cross-track deviation keeps growing unchecked
anyway, that separately answers the activation-vs-weighting question
regardless of proxy precision).
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/crosstrack_diag.csv'
DRIVE_VX = 0.4
GATE_X = 1.0
ARC_RADIUS = 2.0
RAMP_TOP_X = 40.9702
RAMP_BOTTOM_X = 28.2987
RAMP_APPROACH_X = RAMP_TOP_X + 0.5
RAMP_EXIT_X = RAMP_BOTTOM_X - 2.0

# From ramp_mppi_params.yaml, hardcoded here since this diagnostic reads
# live /ramp_state (which selects the profile) rather than querying the
# controller's own live parameters - kept in sync manually, flagged if
# this drifts from the yaml.
PROFILE_OFFSET = {
    'APPROACH': 20, 'EXIT': 20, 'ASCENT': 20, 'TRANSITION': 20,
    'DESCENT': 3, 'HALT': 20,
}


def quat_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def build_turn_and_ramp_path(start_x, start_y, radius, ramp_approach_x, ramp_exit_x, stamp, frame='odom', step=0.4):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    pts = []
    dphi = step / radius
    n = max(1, int(round(math.pi / dphi)))
    for i in range(n + 1):
        phi = math.pi * i / n
        x = start_x + radius * math.sin(phi)
        y = start_y + radius * (1.0 - math.cos(phi))
        pts.append((x, y, phi))
    arc_end_x, arc_end_y = pts[-1][0], pts[-1][1]
    taper_dist = math.hypot(ramp_approach_x - arc_end_x, 0.0 - arc_end_y)
    n2 = max(1, int(round(taper_dist / step)))
    for i in range(1, n2 + 1):
        f = i / n2
        x = arc_end_x + f * (ramp_approach_x - arc_end_x)
        y = arc_end_y + f * (0.0 - arc_end_y)
        pts.append((x, y, math.pi))
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
    return path, [(x, y) for (x, y, yaw) in pts], n  # points, arc-point-count


def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


class Diag(Node):
    def __init__(self):
        super().__init__('crosstrack_diag')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_received = False
        self.rows = []
        self.t0 = time.time()
        self.path_pts = None
        self.arc_point_count = None
        self.nearest_idx_max = 0
        self.last_gz_poll = -999.0
        self.last_gz = None
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.follow_client = ActionClient(self, FollowPath, '/follow_path')
        self._last_log = 0.0

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_received = True
        if self.path_pts is not None:
            self._log_tick()

    def wait_for_odom(self, timeout=15.0):
        deadline = time.time() + timeout
        while not self.odom_received and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _log_tick(self):
        now = time.time() - self.t0
        if now - self._last_log < 0.05:
            return
        self._last_log = now

        # nearest path point index (proxy for MPPI's own closest-point
        # tracking), and a monotonic running max (proxy for
        # furthest_reached_path_point).
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path_pts):
            d = math.hypot(self.odom_x - px, self.odom_y - py)
            if d < best_d:
                best_d, best_i = d, i
        self.nearest_idx_max = max(self.nearest_idx_max, best_i)

        # actual cross-track deviation: perpendicular distance to nearest
        # path SEGMENT (not just nearest point), a tighter proxy.
        seg_i = max(0, min(best_i, len(self.path_pts) - 2))
        ax, ay = self.path_pts[seg_i]
        bx, by = self.path_pts[seg_i + 1]
        crosstrack = point_to_segment_dist(self.odom_x, self.odom_y, ax, ay, bx, by)

        offset_thresh = PROFILE_OFFSET.get(self.state, 20)
        critic_active_proxy = self.nearest_idx_max >= offset_thresh

        gz = self.last_gz
        if now - self.last_gz_poll >= 1.0:
            import subprocess
            try:
                out = subprocess.run(
                    ['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'],
                    capture_output=True, text=True, timeout=2).stdout
                idx = out.find('name: "ramp_robot"')
                if idx >= 0:
                    block = out[idx:idx + 400]
                    pos_i = block.find('position {')
                    pos_block = block[pos_i:pos_i + 150]

                    def pf(txt, key):
                        i = txt.find(key + ':')
                        if i < 0:
                            return None
                        j = txt.find('\n', i)
                        return float(txt[i + len(key) + 1:j].strip())
                    gz = (pf(pos_block, 'x'), pf(pos_block, 'y'), pf(pos_block, 'z'))
                    self.last_gz = gz
            except Exception:
                pass
            self.last_gz_poll = now

        row = {
            't': round(now, 3), 'ramp_state': self.state, 'pitch': round(self.pitch, 2),
            'offset_threshold': offset_thresh, 'nearest_idx': best_i,
            'furthest_idx_proxy': self.nearest_idx_max, 'arc_point_count': self.arc_point_count,
            'critic_active_proxy': critic_active_proxy, 'crosstrack_dev_m': round(crosstrack, 4),
            'odom_x': round(self.odom_x, 3), 'odom_y': round(self.odom_y, 3),
        }
        if gz:
            row['gz_x'], row['gz_y'], row['gz_z'] = (round(v, 3) for v in gz)
        self.rows.append(row)

    def hard_stop(self, n=15):
        stop = Twist()
        for _ in range(n):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_to(self, target_x, vx, timeout=400.0):
        cmd = Twist()
        cmd.linear.x = vx
        deadline = time.time() + timeout
        going_forward = target_x >= self.odom_x
        try:
            while time.time() < deadline:
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                if going_forward and self.odom_x >= target_x:
                    break
                if not going_forward and self.odom_x <= target_x:
                    break
        finally:
            self.hard_stop()

    def follow_arc_and_ramp(self, timeout=240.0):
        self.follow_client.wait_for_server(timeout_sec=10.0)
        start_x, start_y = self.odom_x, self.odom_y
        path, pts, arc_n = build_turn_and_ramp_path(
            start_x, start_y, ARC_RADIUS, RAMP_APPROACH_X, RAMP_EXIT_X,
            self.get_clock().now().to_msg())
        self.path_pts = pts
        self.arc_point_count = arc_n
        self.nearest_idx_max = 0
        self.get_logger().info(
            f'FollowPath arc+ramp: {len(pts)} total points, arc phase ends at point {arc_n}, '
            f'start=({start_x:.2f},{start_y:.2f})')

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

        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if rfut.done():
                    break
        finally:
            self.hard_stop()
        action_result = None
        if rfut.done():
            try:
                action_result = rfut.result()
            except Exception:
                pass
        success = bool(action_result is not None and action_result.status == GoalStatus.STATUS_SUCCEEDED)
        self.get_logger().info(f'leg3 action_reported_success={success} timed_out={not rfut.done()}')
        return success


def main():
    rclpy.init()
    node = Diag()
    try:
        node.wait_for_odom()
        node.get_logger().info(f'STAGE B DIAG START - odom_x={node.odom_x:.2f} state={node.state}')

        ok = node.follow_arc_and_ramp()
        node.get_logger().info(f'leg3 done: ok={ok} odom=({node.odom_x:.2f},{node.odom_y:.2f}) state={node.state}')

        node.get_logger().info('leg4: return to gate (open-loop, no steering correction)...')
        node.path_pts = None  # stop logging against leg3's path during leg4
        node.drive_to(GATE_X, DRIVE_VX)
        node.get_logger().info(f'leg4 done: odom=({node.odom_x:.2f},{node.odom_y:.2f}) state={node.state}')
    finally:
        node.hard_stop()

    if node.rows:
        fieldnames = list(node.rows[0].keys())
        for r in node.rows:
            for k in r:
                if k not in fieldnames:
                    fieldnames.append(k)
        with open(OUT_CSV, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in node.rows:
                w.writerow(r)
        print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
