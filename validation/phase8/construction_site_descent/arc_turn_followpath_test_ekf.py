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

# Goal-verification gate (v7 addition): FollowPath's own goal-completion
# check trusts fused EKF position (Nav2 SimpleGoalChecker vs.
# /odometry/filtered), which kept "succeeding" during the loaded
# construction-site stall even though the chassis was physically frozen
# 0.3-0.6m short of the ramp. /verified_progress_m (published by
# motion_stall_detector_node.py) is independent of the EKF - wheel-
# rotation-derived distance, only counted while the LiDAR scan-
# consistency check is not latched - so it can't be fooled by the same
# failure. A leg is only accepted as complete if BOTH FollowPath itself
# reports success AND this independent signal shows the robot actually
# covered most of the intended path length.
VERIFIED_PROGRESS_MIN_FRACTION = 0.85

# ASCENT / drive-leg physical-reality gate (added after the crest-freeze diagnosis, validation/phase8/crest_freeze/README.md).
# The drive legs (1, 2, 4) used to end when the fused EKF x crossed a target - the same "trust the estimate" bug the leg-3
# goal-verification gate fixed for the descent: a robot frozen on a 0.05 m authoring step with its wheels spinning has an EKF
# that keeps advancing (4.5 m in 50 s in the recording) while the chassis does not move. Each drive leg is now also checked
# against Gazebo ground truth: the chassis displacement must agree with the EKF/odometry displacement within the tolerance
# below. Calibration: across 8 earlier healthy round trips (24 leg ends) the disagreement was <= 0.09 m; a freeze shows metres.
# /verified_progress_m is recorded for every leg but is NOT used as the ascent gate: in the freeze recording it kept growing
# (4.3 m in 50 s = 96% of the raw wheel distance) because every stall/resume cycle re-accumulates the ~5 s of spinning that
# precedes the next latch, so it would have passed the frozen robot.
ASCENT_GT_TOLERANCE_M = 0.5


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
        super().__init__('site_traversal_test7')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.margin = 1.0
        self.slip = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_received = False
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
        # v7: reads the EKF-fused estimate (/odometry/filtered) instead of
        # raw /odom - matches what controller_server itself now uses
        # (nav2_params.yaml's odom_topic) for this test's own bookkeeping.
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        # v7: goal-verification gate - independent of the EKF position
        # that fooled FollowPath's own goal-completion check during the
        # loaded stall. See VERIFIED_PROGRESS_MIN_FRACTION above.
        self.verified_progress_m = 0.0
        self.verified_progress_received = False
        self.leg_checks = []
        self.ascent_verified = None
        self.create_subscription(Float32, '/verified_progress_m', self._on_verified_progress, 10)
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
        self.odom_received = True

    def _on_verified_progress(self, msg):
        self.verified_progress_m = msg.data
        self.verified_progress_received = True

    def wait_for_odom(self, timeout=15.0):
        """A fresh node's /odometry/filtered and /verified_progress_m
        subscriptions each need DDS discovery time before their first
        message arrives - a fixed short spin (the previous 1.5s startup
        delay) is not reliable and was observed live to occasionally leave
        self.odom_x at its 0.0 default, building a goal path from the wrong
        start position. Same failure mode found live for
        /verified_progress_m specifically: motion_stall_detector_node.py
        publishes it as a running total since ITS OWN startup (not per
        leg), so if this node's subscription callback hasn't fired even
        once yet when follow_arc_and_ramp() samples start_verified, that
        read silently returns the 0.0 default instead of the real
        (potentially large, e.g. post-Stage-A) cumulative total - producing
        a nonsensical verified_traveled delta once the real first message
        finally arrives moments later. Block on both until each has
        actually been received at least once, instead of guessing a
        duration for either."""
        deadline = time.time() + timeout
        while (not self.odom_received or not self.verified_progress_received) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.odom_received:
            self.get_logger().error(f'wait_for_odom: no /odometry/filtered message received within {timeout}s')
        if not self.verified_progress_received:
            self.get_logger().error(f'wait_for_odom: no /verified_progress_m message received within {timeout}s')

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

    def drive_to(self, target_x, vx, sample_every=2.0, timeout=400.0, leg_name='drive'):
        gz_start = self._gz_now()
        odom_start, verified_start = self.odom_x, self.verified_progress_m
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
        time.sleep(1.0)
        gz_end = self._gz_now()
        if gz_start and gz_end:
            gt_disp, est_disp = gz_end[0] - gz_start[0], self.odom_x - odom_start
            dis = gt_disp - est_disp
        else:
            gt_disp = est_disp = dis = float('nan')
        intended = abs(target_x - odom_start)
        check = dict(leg=leg_name, target_x=target_x, intended_m=round(intended, 2), ekf_disp_m=round(est_disp, 3), gt_disp_m=round(gt_disp, 3),
                     disagreement_m=round(dis, 3), verified_delta_m=round(self.verified_progress_m - verified_start, 2),
                     ok=bool(dis == dis and abs(dis) <= ASCENT_GT_TOLERANCE_M))
        self.leg_checks.append(check)
        self.get_logger().info(f'{leg_name} physical check: ground-truth displacement {gt_disp:.2f} m vs EKF {est_disp:.2f} m -> disagreement {dis:+.2f} m '
                               f'(tolerance {ASCENT_GT_TOLERANCE_M}); verified_progress delta {check["verified_delta_m"]:.2f} m of {intended:.1f} m intended -> ok={check["ok"]}')
        return check['ok']

    def _gz_now(self):
        for _ in range(4):
            g = get_gz_pose()
            if g:
                return g
        return None

    def follow_arc_and_ramp(self, sample_every=0.5, timeout=360.0):
        # v9 (clean-host recheck): raised again, 240.0 -> 360.0. Sixteen clean-host runs
        # (8 unloaded, 8 loaded) put leg 3 at 190-240 s (mean 211, sd 16.5): one loaded run
        # finished at 239.5 s and another was cut off at 240.0 s while still descending
        # normally, so 240 s again sat on the tail of the natural distribution (the same
        # failure the 180 -> 240 raise below was meant to fix). 360 s is mean + ~9 sd.
        # v8 (site-populated session): raised from 180.0. Two consecutive
        # clean-system loaded runs against the populated world both
        # completed the actual physical descent (109-111% verified
        # progress, final position within <1m of target, normal 16deg
        # pitch profile, no HALT thrash) but had FollowPath's own
        # STATUS_SUCCEEDED arrive just after the 180s wall-clock mark -
        # and the pre-props baseline success (session D) used 179.0 of its
        # own 180s budget, so this ceiling was already razor-thin before
        # any props existed. Not a prop-caused failure - a pre-existing
        # test-harness margin that occasionally lost. See "Prop
        # verification" in validation/phase8/construction_site_descent/
        # README.md.
        self.follow_client.wait_for_server(timeout_sec=10.0)
        start_x, start_y = self.odom_x, self.odom_y
        path = build_turn_and_ramp_path(
            start_x, start_y, ARC_RADIUS, RAMP_APPROACH_X, RAMP_EXIT_X,
            self.get_clock().now().to_msg())
        path_length_m = sum(
            math.hypot(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
                       path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y)
            for i in range(1, len(path.poses)))
        self.get_logger().info(
            f'FollowPath arc+ramp: {len(path.poses)} waypoints, start=({start_x:.2f},{start_y:.2f}), '
            f'arc radius={ARC_RADIUS}, target_x={RAMP_EXIT_X:.2f}, path_length={path_length_m:.2f}m')

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
        start_verified = self.verified_progress_m
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

        timed_out = not rfut.done()
        action_result = None
        if rfut.done():
            try:
                action_result = rfut.result()
            except Exception:
                action_result = None
        action_reported_success = bool(action_result is not None
                                        and action_result.status == GoalStatus.STATUS_SUCCEEDED)

        verified_traveled = self.verified_progress_m - start_verified
        verified_fraction = (verified_traveled / path_length_m) if path_length_m > 0 else 0.0
        goal_verified = verified_fraction >= VERIFIED_PROGRESS_MIN_FRACTION

        # The actual fix: FollowPath's own report is necessary but no
        # longer sufficient. A leg only counts as complete if the
        # independent, EKF-free verified-progress signal also confirms
        # the chassis covered most of the intended path - this is what
        # would have caught the loaded stall (FollowPath reported
        # success there; verified progress would not have cleared the
        # gate, since the chassis never moved after the freeze).
        ok = (not aborted_here) and (not timed_out) and action_reported_success and goal_verified

        self.get_logger().info(
            f'leg goal-verification: path_length={path_length_m:.2f}m '
            f'verified_traveled={verified_traveled:.2f}m ({verified_fraction * 100:.0f}%, '
            f'need >={VERIFIED_PROGRESS_MIN_FRACTION * 100:.0f}%) '
            f'action_reported_success={action_reported_success} timed_out={timed_out} -> ok={ok}')

        self.sample(tag='followpath_arc_ramp_done' + ('_ABORTED' if aborted_here else '') + ('' if ok else '_UNVERIFIED'))
        return ok


class SystemExit_AscentUnverified(Exception):
    pass


def main():
    guard = hg.verify()            # exits 2 with a loud banner BEFORE any motion if the host/sim is not healthy
    monitor = hg.HealthMonitor(period=20.0).start()
    rclpy.init()
    node = Test()
    try:
        node.wait_for_odom()
        node.sample(tag='start')

        node.get_logger().info('leg 1: gate -> laydown area...')
        ok1 = node.drive_to(LAYDOWN_END_X, DRIVE_VX, leg_name='leg 1')
        node.get_logger().info(f'leg 1 done: odom_x={node.odom_x:.2f} state={node.state} margin={node.margin:.3f} physically_verified={ok1}')

        ok2 = False
        if ok1:
            node.get_logger().info('leg 2: laydown -> up the ramp -> onto elevated level...')
            ok2 = node.drive_to(ELEV_MID_X, DRIVE_VX, sample_every=1.0, leg_name='leg 2 (ramp ascent)')
            node.get_logger().info(f'leg 2 done: odom_x={node.odom_x:.2f} pitch={node.pitch:.2f} state={node.state} physically_verified={ok2}')
        node.ascent_verified = bool(ok1 and ok2)
        if not node.ascent_verified:
            node.get_logger().error('ASCENT NOT PHYSICALLY VERIFIED: the EKF says the leg finished but ground truth disagrees - not attempting leg 3/4')
            raise SystemExit_AscentUnverified()

        node.get_logger().info('leg 3: closed-loop FollowPath arc-turn (elevated level) -> taper onto ramp centreline -> across the ramp...')
        ok = node.follow_arc_and_ramp(sample_every=0.5)
        node.get_logger().info(
            f'leg 3 done: ok={ok} odom_x={node.odom_x:.2f} odom_y={node.odom_y:.2f} pitch={node.pitch:.2f} state={node.state}')

        if not node.aborted and ok:
            node.get_logger().info('leg 4: laydown + access road back to the gate (raw cmd_vel, heading already established)...')
            node.drive_to(GATE_X, DRIVE_VX, leg_name='leg 4 (return)')
            node.get_logger().info(f'leg 4 done: odom_x={node.odom_x:.2f} state={node.state}')
        elif not node.aborted:
            node.get_logger().warn(
                'leg 4 SKIPPED: leg 3 did not pass the goal-verification gate '
                '(FollowPath may have reported success, but verified progress did not) '
                '- descent leg is not honestly complete, not attempting the return leg')
    except SystemExit_AscentUnverified:
        ok = False
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
    print(f'ascent_physically_verified: {node.ascent_verified}')
    for c in node.leg_checks:
        print(f'leg_check: {c}')
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
    health = monitor.stop()
    print(f'HARNESS HEALTH: start={guard} during-run={health}')
    if health['starved']:
        hg.banner('this run executed while the host was STARVED (min RTF %s, max load1 %s); '
                  'its result is INVALID for pass/fail purposes.' % (health['min_rtf'], health['max_load1']))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
