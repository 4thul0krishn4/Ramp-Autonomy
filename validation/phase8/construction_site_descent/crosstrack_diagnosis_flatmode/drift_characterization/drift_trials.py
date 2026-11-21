#!/usr/bin/env python3
"""Repeated-trial driver to characterize leg-4 drift distribution (min/
max/mean/std across N full round trips), not just confirm the flat_mode
offset fix helps. Reuses the exact same path-building, drive_to, and
follow_arc_and_ramp logic as the canonical
arc_turn_followpath_test_ekf.py (copied verbatim below, not
reimplemented). ONE trial per process invocation (N_TRIALS arg here is
actually just this trial's own number, for labeling) - a first version
looped many trials inside one long-lived node/sim session to save
relaunch overhead, live-tested and found to self-corrupt
(controller_server logged ~10,677 "Optimizer reset" events over ~50
minutes, and ground truth itself went physically impossible by trial 2 -
whatever degrades, it degrades the live ROS graph itself). Each trial
now gets a genuinely fresh `ros2 launch` + fresh sim, orchestrated
externally (one shell command per trial: kill stale processes, launch,
wait for active, run this script once). Appends one row to OUT_CSV at
the very end of a successful trial only - a killed/crashed trial writes
nothing, so partial/corrupted data can't silently enter the CSV.
"""
import math, time, csv, sys, subprocess, os
from pathlib import Path as _P
for _d in _P(__file__).resolve().parents:
    if (_d / 'harness' / 'harness_guard.py').exists():
        sys.path.insert(0, str(_d / 'harness')); break
import harness_guard as hg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/drift_trials.csv'
PAYLOAD_LABEL = sys.argv[2] if len(sys.argv) > 2 else 'unknown'
TRIAL_NUM = int(sys.argv[3]) if len(sys.argv) > 3 else 1

WORLD_NAME = 'construction_site'
DRIVE_VX = 0.4
GATE_X = 1.0
LAYDOWN_END_X = 28.0
RAMP_TOP_X = 40.9702
RAMP_BOTTOM_X = 28.2987
ELEV_MID_X = RAMP_TOP_X + 6.0
RAMP_APPROACH_X = RAMP_TOP_X + 0.5
RAMP_EXIT_X = RAMP_BOTTOM_X - 2.0

ARC_RADIUS = 2.0
ABORT_MARGIN = 0.15
ABORT_PITCH_DEG = 45.0
VERIFIED_PROGRESS_MIN_FRACTION = 0.85
MIN_VERIFIED_FRACTION_FOR_LEG4 = 0.5


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
        super().__init__('drift_trial_test')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.margin = 1.0
        self.slip = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_received = False
        self.halt_count = 0
        self.min_margin = 1.0
        self.max_slip = 0.0
        self.aborted = False
        self.verified_progress_m = 0.0
        self.verified_progress_received = False
        self.last_gz = None
        self.last_gz_poll = -999.0
        self.t0 = time.time()
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Float32, '/stability_margin', self._on_margin, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_slip, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Float32, '/verified_progress_m', self._on_verified_progress, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.follow_client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        if msg.data != self.state and msg.data == 'HALT':
            self.halt_count += 1
        self.state = msg.data

    def _on_margin(self, msg):
        self.margin = msg.data
        self.min_margin = min(self.min_margin, msg.data)

    def _on_slip(self, msg):
        self.slip = msg.data
        self.max_slip = max(self.max_slip, msg.data)

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_received = True

    def _on_verified_progress(self, msg):
        self.verified_progress_m = msg.data
        self.verified_progress_received = True

    def wait_for_odom(self, timeout=15.0):
        deadline = time.time() + timeout
        while (not self.odom_received or not self.verified_progress_received) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def hard_stop(self, n=15):
        stop = Twist()
        for _ in range(n):
            try:
                self.cmd_pub.publish(stop)
            except Exception:
                break
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                pass

    def gz_now(self):
        g = get_gz_pose()
        if g:
            self.last_gz = g
        return self.last_gz

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

    # 360 s: clean-host leg-3 durations were 190-240 s (n=16, mean 211, sd 16.5); the old 240 s cap
    # sat on the tail of that distribution and turned slow runs into 'failures'.
    def follow_arc_and_ramp(self, timeout=360.0):
        self.follow_client.wait_for_server(timeout_sec=10.0)
        start_x, start_y = self.odom_x, self.odom_y
        path = build_turn_and_ramp_path(
            start_x, start_y, ARC_RADIUS, RAMP_APPROACH_X, RAMP_EXIT_X,
            self.get_clock().now().to_msg())
        path_length_m = sum(
            math.hypot(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
                       path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y)
            for i in range(1, len(path.poses)))

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
            return False, {}
        rfut = gh.get_result_async()

        start_verified = self.verified_progress_m
        deadline = time.time() + timeout
        aborted_here = False
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.margin < ABORT_MARGIN or abs(self.pitch) > ABORT_PITCH_DEG:
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

        timed_out = not rfut.done()
        action_result = None
        if rfut.done():
            try:
                action_result = rfut.result()
            except Exception:
                pass
        action_reported_success = bool(action_result is not None and action_result.status == GoalStatus.STATUS_SUCCEEDED)
        verified_traveled = self.verified_progress_m - start_verified
        verified_fraction = (verified_traveled / path_length_m) if path_length_m > 0 else 0.0
        goal_verified = verified_fraction >= VERIFIED_PROGRESS_MIN_FRACTION
        ok = (not aborted_here) and (not timed_out) and action_reported_success and goal_verified
        info = {
            'aborted': aborted_here, 'timed_out': timed_out,
            'action_reported_success': action_reported_success,
            'verified_fraction': round(verified_fraction, 3),
        }
        return ok, info


def main():
    trial = TRIAL_NUM
    guard = hg.verify()            # exits 2 with a loud banner BEFORE any motion if the host/sim is not healthy
    monitor = hg.HealthMonitor(period=20.0).start()
    rclpy.init()
    node = Test()
    node.wait_for_odom()
    node.get_logger().info(f'TRIAL {trial} ({PAYLOAD_LABEL}) START - odom_x={node.odom_x:.2f} state={node.state}')

    fieldnames = ['trial', 'payload', 't_start', 'duration_s', 'leg3_ok', 'leg3_aborted',
                  'leg3_timed_out', 'leg3_action_success', 'leg3_verified_frac', 'leg4_ran',
                  'trial_halt_count', 'trial_min_margin', 'trial_max_slip',
                  'leg4_gz_x', 'leg4_gz_y', 'leg4_gz_z', 'leg4_odom_x', 'leg4_odom_y',
                  'guard_rtf_start', 'guard_min_rtf', 'guard_max_load1', 'guard_starved']
    write_header = True
    try:
        with open(OUT_CSV, 'r') as f:
            first = f.readline().strip()
            if first:
                write_header = False
                if first.split(',') != fieldnames:
                    hg.banner(f'{OUT_CSV} has a different column layout than this script writes; '
                              f'use a new output file rather than appending misaligned rows.')
                    sys.exit(2)
    except FileNotFoundError:
        pass

    t_trial_start = time.time()

    node.get_logger().info(f'trial {trial}: leg1 gate->laydown...')
    node.drive_to(LAYDOWN_END_X, DRIVE_VX)
    node.get_logger().info(f'trial {trial}: leg2 laydown->elevated level...')
    node.drive_to(ELEV_MID_X, DRIVE_VX)
    node.get_logger().info(f'trial {trial}: leg3 arc-turn + ramp descent...')
    leg3_ok, leg3_info = node.follow_arc_and_ramp()
    node.get_logger().info(f'trial {trial}: leg3 done ok={leg3_ok} {leg3_info} odom=({node.odom_x:.2f},{node.odom_y:.2f})')

    verified_frac = leg3_info.get('verified_fraction', 0.0) or 0.0
    action_success = bool(leg3_info.get('action_reported_success'))
    leg4_ran = False
    if not node.aborted and action_success and 0.0 <= verified_frac <= 1.5 and verified_frac >= MIN_VERIFIED_FRACTION_FOR_LEG4:
        node.get_logger().info(f'trial {trial}: leg4 return to gate...')
        node.drive_to(GATE_X, DRIVE_VX)
        leg4_ran = True
    elif node.aborted:
        node.get_logger().warn(f'trial {trial}: leg4 SKIPPED (safety abort during leg3)')
    elif not action_success:
        node.get_logger().warn(
            f'trial {trial}: leg4 SKIPPED (leg3 FollowPath action did not report success '
            f'verified_fraction={verified_frac:.3f} - turn outcome untrustworthy, '
            f'not a valid leg-4 drift trial)')
    else:
        node.get_logger().warn(
            f'trial {trial}: leg4 SKIPPED (leg3 verified_fraction={verified_frac:.3f} outside '
            f'trustworthy [{MIN_VERIFIED_FRACTION_FOR_LEG4},1.5] range - the turn essentially '
            f'never happened or progress accounting is unreliable, not a valid leg-4 drift trial)')

    gz = node.gz_now()
    duration = time.time() - t_trial_start
    row = {
        'trial': trial, 'payload': PAYLOAD_LABEL, 't_start': round(t_trial_start, 1),
        'duration_s': round(duration, 1),
        'leg3_ok': leg3_ok, 'leg3_aborted': node.aborted,
        'leg3_timed_out': leg3_info.get('timed_out'),
        'leg3_action_success': leg3_info.get('action_reported_success'),
        'leg3_verified_frac': leg3_info.get('verified_fraction'),
        'leg4_ran': leg4_ran,
        'trial_halt_count': node.halt_count, 'trial_min_margin': round(node.min_margin, 3),
        'trial_max_slip': round(node.max_slip, 3),
        'leg4_gz_x': round(gz[0], 3) if gz else None,
        'leg4_gz_y': round(gz[1], 3) if gz else None,
        'leg4_gz_z': round(gz[2], 3) if gz else None,
        'leg4_odom_x': round(node.odom_x, 3), 'leg4_odom_y': round(node.odom_y, 3),
        'guard_rtf_start': guard.get('rtf'),
    }
    health = monitor.stop()
    row['guard_min_rtf'] = health['min_rtf']
    row['guard_max_load1'] = health['max_load1']
    row['guard_starved'] = health['starved']
    if health['starved']:
        hg.banner(f'trial {trial} ran while the host was STARVED (min RTF {health["min_rtf"]}, '
                  f'max load1 {health["max_load1"]}); its result is INVALID for completion/timing statistics.')
    node.get_logger().info(f'trial {trial} RESULT: {row}')

    with open(OUT_CSV, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            w.writeheader()
        w.writerow(row)

    node.hard_stop()
    node.get_logger().info(f'TRIAL {trial} COMPLETE for payload={PAYLOAD_LABEL}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
