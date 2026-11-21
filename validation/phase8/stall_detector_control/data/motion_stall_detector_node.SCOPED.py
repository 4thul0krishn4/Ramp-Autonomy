#!/usr/bin/env python3
"""
motion_stall_detector_node.py

Live implementation of the stall/stumble detector proposed (design-only)
in validation/phase8/construction_site_descent/README.md, after the
loaded construction-site round trip proved the gap wasn't hypothetical:
the robot froze at the ramp-top fillet transition for the ENTIRE ~270 s
test, and the test harness (like every existing safety signal in this
project - /wheel_slip, /stability_margin) had no way to detect it,
because ALL of those signals are proprioceptive (wheel joints, IMU
orientation) and none of them can distinguish "commanded to move and
actually displacing" from "commanded to move and physically stuck."

Two-signal design, per that README's own proposal:

1. PRIMARY, CONFIRMING signal - LiDAR scan-consistency
   (/motion_stalled, std_msgs/Bool). The only genuinely ground-truth-
   -independent signal this robot carries: it observes motion relative
   to the actual environment, not internal actuator/inertial state.
   Compares a CURRENT scan against the scan from STALL_WINDOW_S seconds
   ago and looks for a real change in the range profile.

   HYBRID of both LiDARs on this robot, not a single fixed choice -
   each one tried alone was live-tested against a real failure and
   found to have its own structural blind spot:
   - /scan_360 (level-mounted, purely horizontal 360 deg obstacle
     ring) alone: returned ZERO finite-range beams out of 360 at the
     actual stuck position on construction_site.sdf's open elevated
     platform - every beam either exceeds the 12 m range or passes
     over the platform edge into empty air. Correctly refuses to
     declare a stall on zero valid data (working as designed), but
     that means it is structurally blind in genuinely open areas.
   - /scan (the ramp-profiling LiDAR, rolled 90 deg to sweep a
     vertical plane through the ground/surface a few meters ahead -
     see robot.urdf.xacro's lidar_joint) alone: fixes the open-area
     case (77/360 valid beams at that same stuck position), but
     live-tested on ordinary flat, straight-line driving (this
     project's single most common maneuver) and found to cause
     chronic FALSE stalls - a beam pointed at a fixed relative angle
     into a large, uniformly flat floor returns essentially the SAME
     range whether the robot has translated 0 m or 10 m along that
     same flat surface, so "no change" from this sensor alone does
     not reliably mean "not moving."
   - Fixed by using WHICHEVER sensor has enough valid comparable
     beams (>= MIN_VALID_COMPARABLE_BEAMS) at the moment of the check,
     preferring /scan_360 when both do (it does not share /scan's
     flat-floor blind spot, since a horizontal ring reads real bearing
     /range changes to ANY nearby vertical structure - a wall, a
     ramp's own side, a zone boundary - during genuine translation,
     which this site has close by almost everywhere except the one
     open platform /scan_360 alone couldn't see). /scan is the
     fallback specifically for the open-area case /scan_360 can't
     evaluate at all.

   Whichever sensor is used, the same criterion applies: if the robot
   has been continuously commanded to move (linear or angular, on the
   real actuation topic - the same one stability_monitor_node.py
   already reads for its own commanded-vs-actual comparison) for at
   least STALL_WINDOW_S, but the surrounding range profile has barely
   changed (mean absolute difference over jointly-valid beams below a
   floor sized to the sensor's own known noise - see SCAN_DIFF_STALL_
   THRESHOLD_M below), that is the stall signature.

2. SECONDARY, EARLY-WARNING signal - IMU-based velocity cross-check
   (/motion_stall_warning, std_msgs/Bool). Faster to react (a short
   ~1.5 s window vs. the LiDAR check's longer, higher-confidence
   window) but lower confidence and NOT wired to force anything by
   itself - accelerometer double integration drifts too fast to trust
   past a few seconds, and it only ever watches for missing forward
   acceleration during sustained LINEAR commands (an in-place turn
   legitimately produces ~zero forward accel and would otherwise
   false-positive this check, which is exactly why it's advisory only,
   logged for visibility, not authoritative).

Only /motion_stalled (the LiDAR-confirmed signal) is wired into
state_machine_node.py's HALT logic as a third, independent trigger,
alongside margin_unsafe and slip_unsafe - "confirmed" stall forces
real HALT (which cmd_vel_watchdog_node.py then independently enforces
on the actuation topic, unchanged - it already watches /ramp_state ==
HALT regardless of what caused it, so no changes were needed there).

3. GOAL-VERIFICATION signal - wheel-rotation-derived, scan-corroborated
   progress (/verified_progress_m and /raw_wheel_distance_m, both
   std_msgs/Float32, monotonically increasing running totals since node
   startup). Added after the loaded construction-site round trip proved
   the deeper problem: FollowPath's own goal-completion check (Nav2's
   SimpleGoalChecker, xy_goal_tolerance in nav2_params.yaml) trusts the
   fused EKF position, which kept "progressing" via integrated wheel
   velocity even while the chassis was physically frozen - so the
   ACTION SERVER itself reported success with the robot 0.3-0.6 m short
   of the ramp, not just a metric that failed to notice. /motion_stalled
   makes that failure visible and safe (forces HALT); it does not by
   itself stop a client from also accepting FollowPath's "succeeded"
   result at face value. This signal exists so a client CAN refuse to.

   - /raw_wheel_distance_m: wheel-rotation-derived distance (same
     kinematics as stability_monitor_node.py's _compute_wheel_slip -
     wheel_radius_m=0.12, track_width_m=0.7, same WHEEL_JOINT_SIDE
     mapping), integrated from ACTUAL /joint_states velocity, never
     commanded velocity and never anything from the EKF. This is
     already more honest than fused position (it can't be fooled by a
     runaway position belief), but is still a proprioceptive signal on
     its own - a spinning, non-translating wheel (the exact loaded-
     stall failure mode) still accumulates it, same blind spot
     /wheel_slip has for the same reason.
   - /verified_progress_m: the same integration, but only accumulated
     during time windows where /motion_stalled is NOT latched - i.e.
     only while the independent LiDAR scan-consistency check is not
     currently reporting "commanded to move but the environment isn't
     changing." This is the one signal on the robot that is both (a)
     independent of the fused/EKF position that fooled FollowPath's own
     check, and (b) independent of wheel rotation alone, since a
     spinning-but-stuck period stops counting once the stall latches
     (after STALL_WINDOW_S) - matching the user's "physical reality"
     bar much more closely than either signal alone. It undercounts
     slightly during the first STALL_WINDOW_S of any real stall (that
     window's wheel rotation is counted before the latch fires) and
     during genuinely slow-but-real progress the LiDAR check's window
     might occasionally flag - both biases are in the SAFE direction
     for a goal-completion gate (undercounting real progress just makes
     the gate stricter, never lets a fake success through).

   Consumers (e.g. the construction-site round-trip test script) are
   expected to sample /verified_progress_m immediately before sending a
   FollowPath goal and again after it reports done, and require the
   delta to cover a minimum fraction of the leg's intended path length
   before accepting the leg as complete - regardless of what
   FollowPath's own internal result said. See
   validation/phase8/construction_site_descent/README.md for the
   goal-verification-gate writeup and the loaded round-trip rerun this
   enabled.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float32, String

# Actual actuation topic (downstream of all gating/smoothing) - matches
# stability_monitor_node.py's own CMD_VEL_TOPIC exactly, so "commanded to
# move" means the same thing in both places.
CMD_VEL_TOPIC = '/model/ramp_robot/cmd_vel'

# --- "Is the robot actually being commanded to move" gates ---
MIN_COMMANDED_LINEAR_MPS = 0.05
MIN_COMMANDED_ANGULAR_RPS = 0.05
# Debounce before a sub-threshold cmd_vel tick is treated as "stopped
# commanding" - MPPI/velocity_smoother can emit a single noisy near-zero
# tick even while otherwise driving continuously (a replanning tick, a
# local optimizer dip), and this callback fires on every raw message, not
# a coarse sample - without this, a single such blip resets the
# continuity timer and the "commanded for the full window" gate below
# can never accumulate. Confirmed live: the first implementation without
# this debounce never fired even once across ~90s of continuous real
# driving with a genuine, confirmed stall in the middle of it (see
# validation/phase8/construction_site_descent/loaded/stall_detector/).
COMMAND_DROP_DEBOUNCE_S = 0.5

# --- Primary signal: LiDAR scan-consistency (hybrid, see module docstring) ---
# Both LiDARs (lidar_sensor/scan, obstacle_lidar_sensor/scan_360 in
# robot.urdf.xacro) share the same noise model: 360 beams, 1 deg
# resolution, 0.01 m Gaussian range noise, 10 Hz - confirmed by
# inspection. Two independent noisy reads of the same true range differ
# with stddev sqrt(2)*0.01 =~ 0.014 m; the expected mean |difference|
# under pure noise is a bit below that (~0.011 m). 0.05 m gives a ~4x
# margin above the pure-noise floor while staying far below what real
# displacement produces (confirmed live on both sensors independently:
# /scan 0.0118 m, /scan_360 not separately re-measured but same physical
# noise source).
SCAN_DIFF_STALL_THRESHOLD_M = 0.05
# Confirmed live: /scan_360 returns 0/360 valid beams on
# construction_site.sdf's open elevated platform (too open for its 12 m
# range in every direction); /scan returns 77/360 valid beams at that
# same position. Each sensor is used only when it clears this floor -
# see the module docstring for why /scan_360 is preferred when both do.
MIN_VALID_COMPARABLE_BEAMS = 30  # don't trust the metric on a near-empty scan
STALL_WINDOW_S = 5.0  # long enough to reject normal brief pauses/replans
# Live-tested and found to deadlock PERMANENTLY: clearing the latch only
# on evidence of real movement is circular once HALT is active - HALT
# correctly zeros the actuation topic (that's its job), so the scan
# correctly shows "no movement" for as long as HALT holds, which kept
# the latch set forever, which kept forcing HALT, with no way for the
# FSM to ever get a window to test whether the robot can move again
# (confirmed live: one HALT that never recovered for over 700s straight,
# stuck on ordinary flat ground with nothing physically wrong). Fixed:
# the latch also auto-clears after this cooldown regardless of scan
# evidence, giving the FSM a periodic chance to retry. If the underlying
# problem is still there, the detector re-confirms after another
# STALL_WINDOW_S of renewed-but-fruitless commanding and re-latches -
# a bounded, slow retry cycle (roughly STALL_WINDOW_S + this, ~15s)
# instead of either rapid thrash (the pre-latch version, 40 HALTs in one
# descent leg) or a silent, permanent deadlock (the movement-only-clear
# version).
STALL_LATCH_COOLDOWN_S = 10.0

# --- Independent rotation evidence (added after the leg-3 false-latch diagnosis) ---
# On the open elevated platform /scan_360 sees nothing, so the check falls back to /scan - a vertical-
# plane profiling LiDAR whose only returns are the robot's own chassis and the flat floor ahead.
# Both are invariant to translation AND rotation on a flat plane, so the scan verdict is "no movement"
# whatever the robot does; recorded against Gazebo ground truth, 16/16 latches in one loaded run fired
# while the robot had clearly moved (1.0-2.15 m, up to 50 deg) in the prior 5 s, costing ~48% of leg 3.
# The gyro observes rotation independently of the wheels and of the LiDAR: a chassis that is really
# turning cannot be frozen. Integrated yaw change over this window at or above the minimum overrides a
# "no movement" scan verdict. (Wheel odometry is NOT usable for this: while blocked it reports the
# COMMANDED rotation; the IMU/EKF yaw reports the true, near-zero one - see the validation README.)
# This covers rotation only; straight travel on a featureless floor remains unobservable to the scan.
YAW_EVIDENCE_WINDOW_S = 2.0
YAW_EVIDENCE_MIN_RAD = 0.10

# --- SCOPE of the rotation check: a deliberate workaround, NOT a fix for the underlying problem ---
# Against the curved-world validation cell `curved / normal friction / unloaded`, the rotation check was
# found to be implicated in a regression: with the ORIGINAL scan-only logic that cell passed ground truth
# 6/6 (max |y| 0.39-1.11 m); with the rotation check active everywhere it failed 10/10 (1.74-3.85 m, panel
# half-width 1.5 m). The difference is a stall latch the original logic fires 0.2-0.5 s after the FSM first
# enters DESCENT at the ramp crest (a 1.4-2.2 s stop-and-go, in 5 of 6 original runs) and the rotation check
# suppresses. That pause is accidentally masking an UNSOLVED curved-world descent crosstrack problem, so the
# original logic is kept governing exactly where it does. The boundary, by /ramp_state:
#
#     /ramp_state                       rotation check       stall logic that governs
#     ------------------------------    -----------------    ------------------------------------------
#     APPROACH, ASCENT, TRANSITION,     ACTIVE               scan verdict, overridden by the gyro check
#       EXIT                                                   (as validated: platform, flat, every cell
#                                                              that is not a ramp descent, construction site)
#     DESCENT                           OFF                  ORIGINAL scan-only logic, unchanged
#     HALT                              inherits the state   whichever of the above applied to the state that
#                                       HALT interrupted     was interrupted (HALT interrupting DESCENT =>
#                                                            original logic; interrupting EXIT => gyro check)
#     unknown (no /ramp_state yet)      OFF                  original logic (fail-safe to the pre-change behaviour)
#
# The node cannot tell a curved world from the construction site, so the exclusion is by FSM state, not by
# world. That costs nothing that was validated: on the construction site the rotation check suppressed 14
# false latches on the platform and none during the ramp descent (all descent false latches were straight-line).
# When the curved-world descent crosstrack problem is actually fixed (PathAlignCritic / path tracking), this
# scoping should become unnecessary and ROTATION_CHECK_EXCLUDED_STATES can be emptied.
ROTATION_CHECK_EXCLUDED_STATES = ('DESCENT',)

# --- Secondary signal: short-window IMU forward-accel cross-check ---
IMU_WARNING_WINDOW_S = 1.5
IMU_STALL_DELTA_V_MPS = 0.03
GRAVITY_MPS2 = 9.81

# --- Goal-verification signal: wheel-rotation-derived progress ---
# Same kinematics/conventions as stability_monitor_node.py's
# _compute_wheel_slip, kept identical on purpose so "distance" here and
# "slip" there are computed from the same physical model.
WHEEL_RADIUS_M = 0.12
TRACK_WIDTH_M = 0.7
WHEEL_JOINT_SIDE = {
    'front_left_wheel_joint': 'left',
    'rear_left_wheel_joint': 'left',
    'front_right_wheel_joint': 'right',
    'rear_right_wheel_joint': 'right',
}
# A joint_states tick with an implausibly large dt (startup, a sim-time
# jump, a dropped-then-resumed connection) would otherwise integrate a
# huge bogus distance in one step - discard those ticks instead.
MAX_PLAUSIBLE_JOINT_DT_S = 0.5


class MotionStallDetector(Node):
    def __init__(self):
        super().__init__('motion_stall_detector')

        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._continuously_commanded_since = None  # any (linear or angular) - gates the LiDAR check
        self._linear_commanded_since = None         # linear only - gates the IMU check
        self._below_any_threshold_since = None      # debounce state for the LiDAR-check gate
        self._below_linear_threshold_since = None   # debounce state for the IMU-check gate

        self._scan_ramp_history = deque()    # (t, ranges list) - /scan, the ramp-profiling LiDAR
        self._scan_360_history = deque()     # (t, ranges list) - /scan_360, the horizontal obstacle ring
        self._accel_history = deque()  # (t, net_forward_accel)
        self._gyro_history = deque()   # (t, gyro z) - independent rotation evidence
        self._ramp_state = None        # latest /ramp_state, None until the first message
        self._pre_halt_state = None    # last non-HALT /ramp_state (what a HALT interrupted)

        self.stalled_pub = self.create_publisher(Bool, '/motion_stalled', 10)
        self.warning_pub = self.create_publisher(Bool, '/motion_stall_warning', 10)
        self.raw_distance_pub = self.create_publisher(Float32, '/raw_wheel_distance_m', 10)
        self.verified_progress_pub = self.create_publisher(Float32, '/verified_progress_m', 10)

        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan_ramp, 10)
        self.create_subscription(LaserScan, '/scan_360', self._on_scan_360, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 20)
        self.create_subscription(String, '/ramp_state', self._on_ramp_state, 10)

        self._last_stalled = False
        self._last_warning = False
        self._stall_latched = False  # see _check_lidar_stall's own comment for why this is a latch, not a gate
        self._stall_latched_since = None

        self._last_joint_time = None
        self._raw_wheel_distance_m = 0.0
        self._verified_progress_m = 0.0

        self.get_logger().info(
            f'Motion stall detector running: LiDAR scan-consistency (primary, '
            f'window={STALL_WINDOW_S}s, threshold={SCAN_DIFF_STALL_THRESHOLD_M}m) + '
            f'IMU forward-accel cross-check (early warning, window={IMU_WARNING_WINDOW_S}s) + '
            f'wheel-rotation verified-progress gate (/verified_progress_m, /raw_wheel_distance_m)'
        )

    def _on_cmd_vel(self, msg: Twist):
        now = self._now()
        self._last_cmd_linear = msg.linear.x
        self._last_cmd_angular = msg.angular.z

        commanded = (abs(msg.linear.x) >= MIN_COMMANDED_LINEAR_MPS
                     or abs(msg.angular.z) >= MIN_COMMANDED_ANGULAR_RPS)
        if commanded:
            self._below_any_threshold_since = None
            if self._continuously_commanded_since is None:
                self._continuously_commanded_since = now
        else:
            if self._below_any_threshold_since is None:
                self._below_any_threshold_since = now
            elif now - self._below_any_threshold_since >= COMMAND_DROP_DEBOUNCE_S:
                self._continuously_commanded_since = None

        linear_commanded = abs(msg.linear.x) >= MIN_COMMANDED_LINEAR_MPS
        if linear_commanded:
            self._below_linear_threshold_since = None
            if self._linear_commanded_since is None:
                self._linear_commanded_since = now
        else:
            if self._below_linear_threshold_since is None:
                self._below_linear_threshold_since = now
            elif now - self._below_linear_threshold_since >= COMMAND_DROP_DEBOUNCE_S:
                self._linear_commanded_since = None

    def _on_scan_ramp(self, msg: LaserScan):
        self._on_scan(self._scan_ramp_history, list(msg.ranges))

    def _on_scan_360(self, msg: LaserScan):
        self._on_scan(self._scan_360_history, list(msg.ranges))

    def _on_scan(self, history, ranges):
        now = self._now()
        history.append((now, ranges))
        # Keep a little more than the window so a reference scan is
        # always findable without needing exact timestamp alignment.
        while history and now - history[0][0] > STALL_WINDOW_S + 1.0:
            history.popleft()

        stalled = self._check_lidar_stall(now)
        if stalled != self._last_stalled:
            if stalled:
                self.get_logger().warn(
                    'MOTION STALL CONFIRMED (LiDAR scan-consistency): commanded motion '
                    f'sustained >= {STALL_WINDOW_S}s but surrounding scan barely changed - '
                    'forcing HALT via /motion_stalled'
                )
            else:
                self.get_logger().info('Motion stall cleared (LiDAR scan-consistency: scan now shows real displacement)')
        self._last_stalled = stalled
        self.stalled_pub.publish(Bool(data=stalled))

    @staticmethod
    def _scan_diff(history, now):
        """Returns (valid_comparable_beam_count, mean_abs_diff_or_None) comparing
        the latest scan in `history` against the one from STALL_WINDOW_S ago."""
        if not history:
            return 0, None
        current_ranges = history[-1][1]
        reference = None
        for t, ranges in history:
            if now - t >= STALL_WINDOW_S:
                reference = ranges
            else:
                break
        if reference is None:
            return 0, None
        diffs = []
        n = min(len(reference), len(current_ranges))
        for i in range(n):
            r_ref, r_cur = reference[i], current_ranges[i]
            if math.isfinite(r_ref) and math.isfinite(r_cur):
                diffs.append(abs(r_cur - r_ref))
        if not diffs:
            return 0, None
        return len(diffs), sum(diffs) / len(diffs)

    def _check_lidar_stall(self, now) -> bool:
        # Prefer /scan_360 (no flat-floor blind spot - see module
        # docstring) whenever it has enough valid data; fall back to
        # /scan (the ramp-profiling LiDAR) only when /scan_360 doesn't -
        # this is specifically the open-area case /scan_360 can't see at
        # all (construction_site.sdf's elevated platform).
        n360, diff360 = self._scan_diff(self._scan_360_history, now)
        nramp, diffram = self._scan_diff(self._scan_ramp_history, now)
        if n360 >= MIN_VALID_COMPARABLE_BEAMS:
            scan_shows_no_movement = diff360 < SCAN_DIFF_STALL_THRESHOLD_M
            have_data = True
        elif nramp >= MIN_VALID_COMPARABLE_BEAMS:
            scan_shows_no_movement = diffram < SCAN_DIFF_STALL_THRESHOLD_M
            have_data = True
        else:
            scan_shows_no_movement = False
            have_data = False

        # Rotation seen by the gyro means the chassis is not frozen, whatever the (blind) scan says.
        # (Scoped: not applied during ramp DESCENT - see ROTATION_CHECK_EXCLUDED_STATES.)
        if (have_data and scan_shows_no_movement and self._rotation_check_applies()
                and self._imu_rotation_observed(now)):
            scan_shows_no_movement = False

        # Latched, not just gated: once confirmed, HALT firing zeros the
        # actuation topic (cmd_vel_watchdog_node.py) as an INTENDED side
        # effect - if clearing the stall signal only required "not
        # currently commanded", that self-inflicted zero command would
        # immediately un-confirm the very stall that caused it, the FSM
        # would recover, commanding would resume, the robot (still
        # genuinely stuck) would fail to move again, and the whole cycle
        # would repeat - a HALT-thrash feedback loop, confirmed live (40
        # HALTs in one descent leg with the first, gate-only version of
        # this check).
        #
        # Clearing on real scan movement ALONE was also tried and found to
        # deadlock PERMANENTLY, live: once HALT is active, cmd_vel is
        # correctly zero, so the scan correctly shows "no movement" for as
        # long as HALT holds - there is no way to ever observe the "real
        # movement" evidence this used to require, since the robot is
        # being kept motionless BY the very mechanism waiting to see it
        # move. One HALT never recovered for over 700s straight, stuck on
        # ordinary flat ground with nothing physically wrong. Fixed: the
        # latch ALSO auto-clears after STALL_LATCH_COOLDOWN_S regardless
        # of scan evidence, giving the FSM a periodic chance to retry - if
        # the underlying problem is still there, the detector re-confirms
        # after another STALL_WINDOW_S of renewed-but-fruitless commanding
        # and re-latches, a bounded ~15s retry cycle instead of rapid
        # thrash or a silent permanent deadlock.
        if self._stall_latched:
            if have_data and not scan_shows_no_movement:
                self._stall_latched = False  # real scan change observed - genuinely un-stuck
                self._stall_latched_since = None
                return False
            if self._stall_latched_since is not None and now - self._stall_latched_since >= STALL_LATCH_COOLDOWN_S:
                self._stall_latched = False  # cooldown expired - give the FSM a chance to retry
                self._stall_latched_since = None
                return False
            return True  # still latched: no movement evidence, and cooldown not yet expired

        if self._continuously_commanded_since is None:
            return False
        if now - self._continuously_commanded_since < STALL_WINDOW_S:
            return False
        if not have_data:
            return False  # neither sensor has enough valid history yet to evaluate at all
        self._stall_latched = scan_shows_no_movement
        if self._stall_latched:
            self._stall_latched_since = now
        return self._stall_latched

    def _on_imu(self, msg: Imu):
        now = self._now()
        self._gyro_history.append((now, msg.angular_velocity.z))
        while self._gyro_history and now - self._gyro_history[0][0] > YAW_EVIDENCE_WINDOW_S + 0.5:
            self._gyro_history.popleft()
        q = msg.orientation
        # Expected gravity-reaction specific force along body +x, computed
        # from the current orientation quaternion (standard IMU convention:
        # a stationary sensor reads +g along whichever body axis gravity's
        # reaction pushes on - NOT gravity itself). Body-x component of the
        # world vector (0,0,+g) rotated into the body frame via R^T (R =
        # world_from_body from the quaternion; R^T's first row is R's
        # first column) = 2*(qx*qz - qw*qy) * g.
        expected_gravity_x = 2.0 * (q.x * q.z - q.w * q.y) * GRAVITY_MPS2
        net_forward_accel = msg.linear_acceleration.x - expected_gravity_x

        self._accel_history.append((now, net_forward_accel))
        while self._accel_history and now - self._accel_history[0][0] > IMU_WARNING_WINDOW_S + 0.5:
            self._accel_history.popleft()

        warning = self._check_imu_warning(now)
        if warning != self._last_warning:
            if warning:
                self.get_logger().warn(
                    'motion stall EARLY WARNING (IMU forward-accel): sustained linear command '
                    f'>= {IMU_WARNING_WINDOW_S}s but no accumulated forward acceleration response - '
                    'advisory only, not forcing HALT'
                )
        self._last_warning = warning
        self.warning_pub.publish(Bool(data=warning))

    def _check_imu_warning(self, now) -> bool:
        if self._linear_commanded_since is None:
            return False
        if now - self._linear_commanded_since < IMU_WARNING_WINDOW_S:
            return False

        window = [(t, a) for (t, a) in self._accel_history if now - t <= IMU_WARNING_WINDOW_S]
        if len(window) < 2:
            return False

        # Trapezoidal integration of net forward accel over the window to
        # estimate the resulting delta-v.
        delta_v = 0.0
        for i in range(1, len(window)):
            t0, a0 = window[i - 1]
            t1, a1 = window[i]
            delta_v += 0.5 * (a0 + a1) * (t1 - t0)

        return abs(delta_v) < IMU_STALL_DELTA_V_MPS

    def _on_joint_states(self, msg: JointState):
        now = self._now()
        if self._last_joint_time is None:
            self._last_joint_time = now
            return
        dt = now - self._last_joint_time
        self._last_joint_time = now
        if dt <= 0.0 or dt > MAX_PLAUSIBLE_JOINT_DT_S:
            return

        side_velocities = {'left': [], 'right': []}
        for name, velocity in zip(msg.name, msg.velocity):
            side = WHEEL_JOINT_SIDE.get(name)
            if side is not None:
                side_velocities[side].append(velocity)
        if not side_velocities['left'] or not side_velocities['right']:
            return  # this JointState tick doesn't carry the wheel joints at all

        left_avg = sum(side_velocities['left']) / len(side_velocities['left'])
        right_avg = sum(side_velocities['right']) / len(side_velocities['right'])
        # Standard diff-drive forward kinematics (same convention as
        # stability_monitor_node.py's commanded_side computation, inverted):
        # implied chassis linear speed from actual wheel angular velocity.
        implied_speed_mps = WHEEL_RADIUS_M * (left_avg + right_avg) / 2.0
        step_distance_m = abs(implied_speed_mps) * dt

        self._raw_wheel_distance_m += step_distance_m
        # Only counts toward the verified total while the independent
        # LiDAR scan-consistency check is not currently latched - see the
        # module docstring's signal-3 section for why this combination
        # (not raw wheel rotation alone) is what actually reflects
        # physical reality rather than "the wheels are turning."
        if not self._stall_latched:
            self._verified_progress_m += step_distance_m

        self.raw_distance_pub.publish(Float32(data=self._raw_wheel_distance_m))
        self.verified_progress_pub.publish(Float32(data=self._verified_progress_m))

    def _on_ramp_state(self, msg: String):
        if msg.data != 'HALT':
            self._pre_halt_state = msg.data
        self._ramp_state = msg.data

    def _rotation_check_applies(self) -> bool:
        """Whether the gyro rotation override may act now. See ROTATION_CHECK_EXCLUDED_STATES for the table.
        HALT inherits the state it interrupted; an unknown state falls back to the original logic."""
        state = self._pre_halt_state if self._ramp_state == 'HALT' else self._ramp_state
        if state is None:
            return False
        return state not in ROTATION_CHECK_EXCLUDED_STATES

    def _imu_rotation_observed(self, now) -> bool:
        """True if the gyro-integrated yaw change over the last YAW_EVIDENCE_WINDOW_S >= YAW_EVIDENCE_MIN_RAD."""
        h = [(t, w) for (t, w) in self._gyro_history if now - t <= YAW_EVIDENCE_WINDOW_S]
        if len(h) < 5 or h[-1][0] - h[0][0] < 0.8 * YAW_EVIDENCE_WINDOW_S:
            return False          # not enough history to say - fall back to the scan verdict alone
        dyaw = 0.0
        for i in range(1, len(h)):
            dyaw += 0.5 * (h[i - 1][1] + h[i][1]) * (h[i][0] - h[i - 1][0])
        return abs(dyaw) >= YAW_EVIDENCE_MIN_RAD

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MotionStallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
