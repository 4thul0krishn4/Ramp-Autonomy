#!/usr/bin/env python3
"""
Phase 8 validation-matrix test cell, EKF + ground-truth follow-up.

Same structure and pass/fail intent as the original
validation/phase8/validation_test.py, kept as a SEPARATE script (the
original is preserved unmodified as the historical, pre-EKF,
odometry-only record - see its own docstring and the README's "Known
gaps" entry for why that matters). Two changes only:

  1. Launched against ramp_autonomy_bringup.launch.py exactly as before,
     but that launch file now includes ekf_node (robot_localization) and
     controller_server now reads /odometry/filtered instead of raw /odom
     - no changes needed in THIS script for that part, it's transparent
       at the launch-graph level. odom_x/odom_yaw read here still comes
       from /odom for continuity with the original script's own semantics
       (this script's own driving decisions - not what the controller
       uses internally, which changed automatically).

  2. NEW: polls Gazebo's own ground-truth model pose throughout (not just
     at the end) - the same technique curved_descent_diagnosis.py already
     used to root-cause the original curved-world tip-over, generalized
     here to EVERY matrix cell instead of a one-off diagnosis. A cell can
     satisfy every original criterion (DESCENT reached, HALT count
     acceptable) purely from wheel-odometry/FSM-state bookkeeping while
     the robot's REAL position has already left the ramp/zone panel - the
     original script had no way to see that. Adds one new, pre-declared
     criterion:

  5. Ground-truth lateral safety: max |ground-truth Y| during the
     descent leg must stay within the ramp/zone panel's own half-width
     (1.5 m - both ramp_world_sharp.sdf and ramp_world_curved.sdf and all
     their TESTONLY friction variants use 3.0 m-wide zone panels,
     confirmed by inspection, not assumed). Exceeding it means the robot
     physically left the panel surface, regardless of what /ramp_state or
     wheel odometry claimed - this is the exact mechanism already proven
     to cause real tip-overs in this project's own history.

Overall verdict = worst of ALL FIVE individual criteria now (still FAIL
beats MARGINAL beats PASS).
"""
import json, math, os, sys, time, subprocess
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'run'
WORLD_NAME = sys.argv[2] if len(sys.argv) > 2 else 'ramp_world_sharp'
ASCENT_TIMEOUT_S = 100.0
HALT_MARGIN_LIMIT = 3     # 1..3 HALTs => MARGINAL
HALT_THRASH_LIMIT = 8     # >=8 HALTs => FAIL (old vx_max approach hit 146)
ZONE_HALF_WIDTH_M = 1.5   # 3.0 m-wide zone panels, both worlds - confirmed by inspection
# Descent path waypoint spacing. Default 0.4 m = the historical path (every earlier run); anything else is built by
# ramp_autonomy_nav.path_source (structural path-density experiments, validation/phase8/path_structural/).
PATH_SPACING = float(os.environ.get('PATH_SPACING', '0.4'))
GT_TRACE = os.environ.get('GT_TRACE', '') not in ('', '0')   # also print the (t, x, y) ground-truth samples
# TURN PROCEDURE (corrected standard for ALL worlds, validation/phase8/path_structural/README.md). The historical procedure turned the
# robot open-loop for 6.6 s exactly where EXIT is first reached, at the lip of the top platform. Recorded ground truth showed: on the
# CURVED crest that turn achieves only ~90 of 180 degrees and displaces the chassis 0.2-1.3 m sideways while the EKF's y stays 0.00
# (wheel forward velocity only), which corrupted every earlier curved-world result; on the SHARP crest it achieves only ~64 degrees
# (little displacement, 0.08 m) so the robot starts the descent facing sideways and never descends (0.05-0.12 m of 6 m). Default now, every world:
# drive PLATFORM_TURN_M further onto the flat top platform, then turn CLOSED-LOOP on the EKF (IMU-fused) yaw to 180 degrees.
# Environment overrides, either direction:
#   CLOSED_LOOP_TURN=0 PLATFORM_TURN_M=0   -> the historical crest open-loop turn (to reproduce old results)
#   PLATFORM_TURN_M=x                      -> different platform drive distance
# ASCENT physical-reality gate (crest-freeze diagnosis, validation/phase8/crest_freeze/README.md): the ascent used to be judged complete
# when /ramp_state reached EXIT (a pitch-based FSM state) - it never checked that the chassis physically got anywhere. A robot frozen on a
# 0.05 m authoring step with spinning wheels reached EXIT and was called an ascent PASS while its EKF/odometry x ran metres ahead of the chassis.
# Now: ground-truth chassis displacement over the ascent must agree with the odometry displacement within this tolerance (calibrated on 24
# healthy leg ends: <= 0.09 m), and (corrected procedure) the platform drive must physically advance the chassis.
ASCENT_GT_TOLERANCE_M = 0.5
CLOSED_LOOP_TURN = (os.environ['CLOSED_LOOP_TURN'] not in ('', '0')) if 'CLOSED_LOOP_TURN' in os.environ else True
PLATFORM_TURN_M = float(os.environ['PLATFORM_TURN_M'] or 0) if 'PLATFORM_TURN_M' in os.environ else 2.4


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

        px = parse_field(pos_block, 'x')
        py = parse_field(pos_block, 'y')
        pz = parse_field(pos_block, 'z')
        return px, py, pz
    except Exception:
        return None


class T(Node):
    def __init__(self):
        super().__init__('validation_test')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.odom_x = 0.0
        self.odom_yaw = 0.0
        self._prev_state = 'APPROACH'
        self.state_log = []
        self.halt_count = 0
        self.reached_descent = False
        self.reached_exit = False
        self.t0 = time.time()
        # Ground-truth tracking (new)
        self.gt_max_abs_y_descent = 0.0
        self.gt_samples_descent = 0
        self.gt_last_poll = -999.0
        self.gt_poll_failures = 0
        self.gt_final_pose = None
        self.gt_trace = []
        self.platform_drive_m = 0.0

        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.ekf_yaw = 0.0
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        if msg.data != self._prev_state:
            t = round(time.time() - self.t0, 2)
            self.state_log.append((t, self._prev_state, '->', msg.data))
            if msg.data == 'HALT':
                self.halt_count += 1
            if msg.data == 'DESCENT':
                self.reached_descent = True
            self._prev_state = msg.data
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    def _on_ekf(self, msg):
        q = msg.pose.pose.orientation
        self.ekf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def turn_closed_loop(self, target_yaw, tol_deg=4.0, wz_max=0.5, timeout_s=40.0):
        """Rotate (counter-clockwise) on the EKF yaw until within tol of target_yaw; returns the final EKF yaw."""
        end = time.time() + timeout_s
        while time.time() < end:
            err = math.atan2(math.sin(target_yaw - self.ekf_yaw), math.cos(target_yaw - self.ekf_yaw))
            if abs(err) < math.radians(tol_deg):
                break
            t = Twist(); t.angular.z = math.copysign(min(wz_max, max(0.12, 0.8 * abs(err))), err)
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.03)
        stop = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.ekf_yaw

    def drive_ground_truth_distance(self, dist_m, vx=0.25, timeout_s=60.0):
        """Drive forward until Gazebo ground truth says the chassis has advanced dist_m (or timeout). A TIMED drive was used first and
        was not enough: a loaded robot on mu 0.45 stalled/HALTed at the crest for ~10 s, the timed drive ended with the robot still on the
        crest, and the turn then happened there (the exact displacement the procedure exists to avoid). Wheel odometry cannot be used to
        measure this (it counts wheel rotation even when the chassis is blocked); ground truth can."""
        p0 = None
        for _ in range(5):
            p0 = get_gz_pose(WORLD_NAME)
            if p0 is not None:
                break
        if p0 is None:
            self.get_logger().warning(f'[{LABEL}] no ground truth for the platform drive - falling back to the timed drive')
            self.drive(vx, 0.0, dist_m / vx)
            self.platform_drive_m = float('nan')
            return
        end = time.time() + timeout_s
        last_poll, moved = 0.0, 0.0
        cmd = Twist(); cmd.linear.x = vx
        while time.time() < end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.03)
            if time.time() - last_poll > 0.7:
                last_poll = time.time()
                p = get_gz_pose(WORLD_NAME)
                if p is not None:
                    moved = math.hypot(p[0] - p0[0], p[1] - p0[1])
                    if moved >= dist_m:
                        break
        stop = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.platform_drive_m = moved
        self.get_logger().info(f'[{LABEL}] platform drive advanced {moved:.2f} m (ground truth) of {dist_m} m requested')

    def spin_for(self, s, period=0.05):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=period)

    def poll_ground_truth(self, track_descent=False, min_interval=1.0):
        now = time.time() - self.t0
        if now - self.gt_last_poll < min_interval:
            return
        self.gt_last_poll = now
        pose = get_gz_pose(WORLD_NAME)
        if pose is None:
            self.gt_poll_failures += 1
            return
        self.gt_final_pose = pose
        if track_descent:
            self.gt_trace.append((round(now, 2), round(pose[0], 3), round(pose[1], 3)))
            self.gt_samples_descent += 1
            self.gt_max_abs_y_descent = max(self.gt_max_abs_y_descent, abs(pose[1]))

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

    def send_path_and_wait(self, path, timeout_sec, target_x, target_tol=0.5):
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
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.poll_ground_truth(track_descent=True)
            if abs(self.odom_x - target_x) < target_tol:
                break
            if rfut.done():
                break
        else:
            gh.cancel_goal_async()
            time.sleep(1.0)


def main():
    rclpy.init()
    node = T()
    node.spin_for(1.0)
    node.client.wait_for_server(timeout_sec=10.0)

    node.get_logger().info(f'[{LABEL}] ascending (budget {ASCENT_TIMEOUT_S:.0f}s)...')
    gz_ascent_start = None
    for _ in range(5):
        gz_ascent_start = get_gz_pose(WORLD_NAME)
        if gz_ascent_start is not None:
            break
    odom_ascent_start = node.odom_x
    was_on_ramp = False
    cmd = Twist(); cmd.linear.x = 0.25
    end = time.time() + ASCENT_TIMEOUT_S
    last_print = 0
    while time.time() < end:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.03)
        node.poll_ground_truth(track_descent=False)
        if abs(node.pitch) > 10.0:
            was_on_ramp = True
        now = time.time()
        if now - last_print > 5.0:
            last_print = now
            node.get_logger().info(f'[{LABEL}] t={now-node.t0:.0f} pitch={node.pitch:.1f} state={node.state}')
        if was_on_ramp and node.state == 'EXIT':
            node.reached_exit = True
            node.get_logger().info(f'[{LABEL}] reached EXIT')
            break
    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.spin_for(2.0)

    verdict_ascent = 'PASS' if node.reached_exit else 'FAIL: DID_NOT_CLIMB'
    ascent_disagreement = float('nan')
    gz_ascent_end = None
    if node.reached_exit:
        for _ in range(5):
            gz_ascent_end = get_gz_pose(WORLD_NAME)
            if gz_ascent_end is not None:
                break
        if gz_ascent_start is not None and gz_ascent_end is not None:
            ascent_disagreement = (gz_ascent_end[0] - gz_ascent_start[0]) - (node.odom_x - odom_ascent_start)
            if abs(ascent_disagreement) > ASCENT_GT_TOLERANCE_M:
                verdict_ascent = (f'FAIL: ASCENT_POSITION_UNVERIFIED (ground-truth displacement {gz_ascent_end[0] - gz_ascent_start[0]:.2f} m vs odometry '
                                  f'{node.odom_x - odom_ascent_start:.2f} m; disagreement {ascent_disagreement:+.2f} m > {ASCENT_GT_TOLERANCE_M} m)')
    descent_distance = 0.0
    verdict_descent = 'N/A (ascent failed)'
    verdict_ground_truth = 'N/A (ascent failed)'

    platform_ok = verdict_ascent == 'PASS'
    if node.reached_exit and platform_ok and PLATFORM_TURN_M > 0:
        node.get_logger().info(f'[{LABEL}] driving {PLATFORM_TURN_M} m further onto the flat platform before turning')
        node.drive_ground_truth_distance(PLATFORM_TURN_M)
        node.spin_for(1.5)
        # If the chassis never got onto the platform (e.g. a 100 kg robot on mu 0.45 stuck at the crest with its wheels spinning - wheel
        # odometry then runs away from ground truth), turning it there is exactly the crest displacement this procedure exists to avoid and
        # produces a fake edge departure. That is a failure to CLIMB, reported as such, not a descent verdict.
        if node.platform_drive_m == node.platform_drive_m and node.platform_drive_m < 0.8 * PLATFORM_TURN_M:
            platform_ok = False
            verdict_ascent = (f'FAIL: DID_NOT_REACH_PLATFORM (ground truth: chassis advanced only {node.platform_drive_m:.2f} m of the '
                              f'{PLATFORM_TURN_M} m platform drive; stuck at the crest although /ramp_state reached EXIT)')
    if node.reached_exit and platform_ok:
        if CLOSED_LOOP_TURN:
            yaw_before = node.ekf_yaw
            node.turn_closed_loop(math.pi)
            node.get_logger().info(f'[{LABEL}] closed-loop turn: ekf yaw {math.degrees(yaw_before):.0f} -> {math.degrees(node.ekf_yaw):.0f} deg')
        else:
            node.drive(0.0, 0.5, 6.6)  # turn 180deg (open loop, historical)
        node.spin_for(1.0)
        node.t0 = time.time()
        node.gt_last_poll = -999.0
        start_x = node.odom_x
        yaw_target = math.pi if (CLOSED_LOOP_TURN or abs(abs(node.odom_yaw) - math.pi) < 0.5) else 0.0
        if PATH_SPACING == 0.4:
            path_down = make_path(node.odom_x - 0.5, node.odom_x - 6.5, 0.4, yaw_target,
                                   node.get_clock().now().to_msg())          # historical path, bit-identical
        else:
            from ramp_autonomy_nav import path_source
            path_down = path_source.make_straight_path(node.odom_x - 0.5, node.odom_x - 6.5, yaw_target,
                                                        node.get_clock().now().to_msg(), spacing=PATH_SPACING)
        node.get_logger().info(f'[{LABEL}] descent path: {len(path_down.poses)} waypoints, spacing {PATH_SPACING} m, '
                               f'start odom (x={node.odom_x:.2f}) y_path=0')
        node.spin_for(0.3)
        node.send_path_and_wait(path_down, 90.0, node.odom_x - 6.0)
        descent_distance = abs(node.odom_x - start_x)
        verdict_descent = 'PASS' if node.reached_descent else 'FAIL: DESCENT_NOT_REACHED'

        if node.gt_samples_descent == 0:
            verdict_ground_truth = 'MARGINAL: NO_GROUND_TRUTH_SAMPLES (gz polling failed every attempt)'
        elif node.gt_max_abs_y_descent > ZONE_HALF_WIDTH_M:
            verdict_ground_truth = (f'FAIL: GROUND_TRUTH_EDGE_DEPARTURE (max |y|={node.gt_max_abs_y_descent:.2f} m '
                                     f'vs panel half-width {ZONE_HALF_WIDTH_M} m, n={node.gt_samples_descent} samples)')
        else:
            verdict_ground_truth = f'PASS (max |y|={node.gt_max_abs_y_descent:.2f} m, n={node.gt_samples_descent} samples)'

    if node.halt_count >= HALT_THRASH_LIMIT:
        verdict_halt = f'FAIL: HALT_THRASH ({node.halt_count} HALTs)'
    elif node.halt_count >= 1:
        verdict_halt = f'MARGINAL ({node.halt_count} HALTs)'
    else:
        verdict_halt = 'PASS (0 HALTs)'

    verdicts = [verdict_ascent, verdict_descent, verdict_halt, verdict_ground_truth]
    if any(v.startswith('FAIL') for v in verdicts):
        overall = 'FAIL'
    elif any(v.startswith('MARGINAL') for v in verdicts):
        overall = 'MARGINAL'
    else:
        overall = 'PASS'

    print(f'\n===== VALIDATION RESULT for {LABEL} (EKF + ground truth) =====')
    print(f'overall_verdict: {overall}')
    print(f'ascent_verdict: {verdict_ascent}')
    print(f'descent_verdict: {verdict_descent}')
    print(f'halt_verdict: {verdict_halt}')
    print(f'ground_truth_verdict: {verdict_ground_truth}')
    print(f'reached_exit: {node.reached_exit}')
    print(f'ascent_gt_vs_odom_disagreement_m: {ascent_disagreement:.3f}')
    print(f'reached_descent: {node.reached_descent}')
    print(f'descent_leg_distance_m: {descent_distance:.2f}')
    print(f'halt_count: {node.halt_count}')
    print(f'final_pitch_deg: {node.pitch:.1f}')
    print(f'gt_max_abs_y_descent_m: {node.gt_max_abs_y_descent:.3f}')
    print(f'gt_samples_descent: {node.gt_samples_descent}')
    print(f'gt_poll_failures: {node.gt_poll_failures}')
    print(f'gt_final_pose: {node.gt_final_pose}')
    print(f'path_spacing_m: {PATH_SPACING}')
    print(f'closed_loop_turn: {CLOSED_LOOP_TURN}')
    print(f'platform_turn_m: {PLATFORM_TURN_M}')
    print(f'platform_drive_advanced_m: {node.platform_drive_m}')
    print(f'ekf_yaw_at_end_deg: {math.degrees(node.ekf_yaw):.1f}')
    if GT_TRACE:
        print('gt_trace_json: ' + json.dumps(node.gt_trace))
    print('state_log:')
    for row in node.state_log:
        print(' ', row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
