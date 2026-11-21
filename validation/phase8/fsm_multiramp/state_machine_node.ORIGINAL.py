#!/usr/bin/env python3
"""
state_machine_node.py

Phase 3 baseline implementation of the 5-state ramp traversal flow:

    APPROACH -> ASCENT -> TRANSITION -> DESCENT -> EXIT

plus a HALT state any of the four can drop into on an unsafe
stability margin (wired properly in phase 6 - see the TODO below).
HALT recovery (phase 7, generalized in a later phase-7 pass) always
returns to whichever state HALT actually interrupted - tracked in
self._pre_halt_state - rather than unconditionally dropping back to
APPROACH (see _update_state's HALT-entry and HALT-exit branches for
the full reasoning, including why a single remembered value is enough
even if HALT fires again shortly after recovering).

Phase 7 bug fix: ASCENT is now guaranteed to have a way out once the
robot has genuinely left the ramp, independent of whether perception's
pre-emption or the IMU pitch-rate spike ever fires (see _update_state's
'ASCENT and not on_ramp' branch for the full story - this was a real,
live-reproduced stuck state, not a hypothetical).

Ramp detection is still primarily the IMU-pitch threshold ("bootstrap
heuristic") for APPROACH->ASCENT and everything except entering
TRANSITION - it's simple, always available, and doesn't depend on
perception being confident. Phase 4 adds ONE break-point-aware
pre-emption on top of it: entering TRANSITION (ASCENT -> TRANSITION)
can now fire early from ramp_classifier_node.py's look-ahead estimate
of the upcoming break point's distance, instead of waiting for the
robot's own pitch to already be changing. The IMU-based check stays as
a fallback/cross-check for whenever perception's confidence is too low
to trust (see PERCEPTION_MIN_CONFIDENCE) - it is not replaced, both
conditions are OR'd together.

Published:
    /ramp_state        (std_msgs/String)  current FSM state, for RViz/logging
    /cmd_vel_scale      (std_msgs/Float32) speed multiplier the local
                        planner (MPPI cost function, phase 5) should
                        apply for the current state

Subscribed:
    /imu                (sensor_msgs/Imu)   pitch angle -> state inference
    /stability_margin    (std_msgs/Float32)  from phase 6's stability
                        monitor; margin below STABILITY_HALT_THRESHOLD
                        forces a transition to HALT regardless of state
    /wheel_slip          (std_msgs/Float32)  from phase 6's stability
                        monitor; ratio above WHEEL_SLIP_HALT_THRESHOLD
                        ALSO forces HALT, independently of margin - a
                        tipping-margin check has nothing useful to say
                        about a wheel sliding on a low-friction surface
                        while the robot stays level, so this is a
                        separate failure mode, not folded into the same
                        scalar.
    /ramp_perception     (ramp_autonomy_msgs/RampPercept) phase 4's
                        look-ahead break-point estimate; used only to
                        pre-empt ASCENT -> TRANSITION (see above)
    /motion_stalled      (std_msgs/Bool) motion_stall_detector_node.py's
                        LiDAR-scan-consistency-confirmed stall signal -
                        forces HALT exactly like margin_unsafe/slip_unsafe,
                        a third independent failure mode (see that node's
                        own docstring: the robot can be commanded to move,
                        wheels rotating consistent with the command, and
                        genuinely not displacing at all - proprioceptive
                        signals alone, including this project's own EKF,
                        cannot detect that; the loaded construction-site
                        round trip proved it wasn't hypothetical, frozen
                        for the entire ~270s test with every existing
                        safety signal reporting healthy the whole time)
"""
import math

import rclpy
from rclpy.node import Node
from ramp_autonomy_msgs.msg import RampPercept
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String


# --- Bootstrap thresholds (phase 3) - IMU fallback stays in place
# alongside phase 4's perception-based pre-emption below ---
PITCH_ON_RAMP_DEG = 5.0          # |pitch| above this => on the slope
TRANSITION_WINDOW_DEG = 2.0      # pitch-rate window counted as "transitioning"

# --- Phase 4: perception-based pre-emptive TRANSITION trigger ---
# breakpoint_distance_m is measured from the LiDAR mount, not robot
# center (~0.35 m ahead of base_link) - a few tens of cm of systematic
# bias is fine for a "start slowing down soon" trigger, not worth
# correcting for here. -1.0 (RampPercept's NO_BREAKPOINT sentinel)
# never satisfies "<= trigger distance", so it's naturally ignored.
BREAKPOINT_TRIGGER_DISTANCE_M = 0.75
PERCEPTION_MIN_CONFIDENCE = 0.5

# --- Speed scaling per state, consumed by the MPPI cost function (phase 5) ---
SPEED_SCALE = {
    'APPROACH': 1.0,
    'ASCENT': 0.6,
    'TRANSITION': 0.35,
    'DESCENT': 0.5,
    'EXIT': 1.0,
    'HALT': 0.0,
}

STABILITY_HALT_THRESHOLD = 0.15  # TODO(phase 6): tune against real margin units

# --- Phase 6: wheel-slip HALT trigger, independent of tip-over margin ---
# slip_ratio = |commanded - actual| / |commanded| per wheel (see
# stability_monitor_node.py) - 0.0 is a wheel tracking its command
# perfectly, 1.0 is a wheel spinning completely free of the commanded
# rate (full loss of traction). 0.5 as a starting threshold is a
# placeholder like STABILITY_HALT_THRESHOLD was before phase 6's live
# test - see the README for what tuning against an actual low-friction
# run showed.
WHEEL_SLIP_HALT_THRESHOLD = 0.5


class RampStateMachine(Node):
    def __init__(self):
        super().__init__('ramp_state_machine')

        self.state = 'APPROACH'
        # See _update_state's HALT-entry and HALT-recovery branches for
        # the full reasoning. Short version: this remembers whatever
        # state was active the instant HALT was entered, so recovery can
        # return to it exactly rather than unconditionally dropping back
        # to APPROACH. A single scalar (not a stack) is provably enough
        # here - HALT episodes cannot nest in this FSM, since the safety
        # check runs first on every tick with an unconditional early
        # return, so a state transition and a HALT-entry/exit can never
        # both happen on the same tick, and this value is only ever
        # written on a non-HALT -> HALT edge and only ever read on a
        # HALT -> non-HALT edge.
        self._pre_halt_state = 'APPROACH'
        self._prev_pitch_deg = 0.0
        self._stability_margin = 1.0  # optimistic default until phase 6 publishes real values
        self._wheel_slip = 0.0        # optimistic default until phase 6 publishes real values
        self._percept_breakpoint_distance = -1.0  # RampPercept's NO_BREAKPOINT sentinel, until first msg arrives
        self._percept_confidence = 0.0
        self._motion_stalled = False  # optimistic default until motion_stall_detector_node.py publishes

        self.state_pub = self.create_publisher(String, '/ramp_state', 10)
        self.speed_scale_pub = self.create_publisher(Float32, '/cmd_vel_scale', 10)

        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(Float32, '/stability_margin', self._on_stability_margin, 10)
        self.create_subscription(Float32, '/wheel_slip', self._on_wheel_slip, 10)
        self.create_subscription(RampPercept, '/ramp_perception', self._on_percept, 10)
        self.create_subscription(Bool, '/motion_stalled', self._on_motion_stalled, 10)

        self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            'Ramp state machine started in APPROACH (IMU fallback + phase 4 break-point pre-emption)'
        )

    def _on_stability_margin(self, msg: Float32):
        self._stability_margin = msg.data

    def _on_wheel_slip(self, msg: Float32):
        self._wheel_slip = msg.data

    def _on_percept(self, msg: RampPercept):
        self._percept_breakpoint_distance = msg.breakpoint_distance_m
        self._percept_confidence = msg.confidence

    def _on_motion_stalled(self, msg: Bool):
        self._motion_stalled = msg.data

    def _on_imu(self, msg: Imu):
        # Extract pitch from quaternion (rotation about Y)
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_deg = math.degrees(math.asin(sinp))
        pitch_rate = pitch_deg - self._prev_pitch_deg
        self._prev_pitch_deg = pitch_deg

        self._update_state(pitch_deg, pitch_rate)

    def _update_state(self, pitch_deg: float, pitch_rate: float):
        # Safety check first: any state can be pre-empted by HALT, from
        # any of three independent failure modes - tipping (low margin),
        # sliding (high wheel slip), or a confirmed physical stall (wheels
        # rotating close to commanded, chassis genuinely not displacing -
        # see motion_stall_detector_node.py, added after the loaded
        # construction-site round trip proved this a real, not
        # hypothetical, gap: frozen for the entire ~270s test with margin
        # and slip both reporting healthy throughout). Different physical
        # causes, checked and reported separately rather than folded into
        # one scalar; any one alone is sufficient to force HALT.
        # TODO(phase 6): consider hysteresis on margin/slip so HALT doesn't
        # chatter right at either threshold.
        margin_unsafe = self._stability_margin < STABILITY_HALT_THRESHOLD
        slip_unsafe = self._wheel_slip > WHEEL_SLIP_HALT_THRESHOLD
        stall_unsafe = self._motion_stalled
        if margin_unsafe or slip_unsafe or stall_unsafe:
            if self.state != 'HALT':
                # HALT-entry edge: capture what we're interrupting. Only
                # fires on the FIRST unsafe tick of an episode (guarded by
                # self.state != 'HALT'), so if margin/slip/stall stay
                # unsafe for many consecutive ticks this is not repeatedly
                # overwritten with 'HALT' itself - see recovery branch below.
                self._pre_halt_state = self.state
                if margin_unsafe:
                    self.get_logger().warn(
                        f'Stability margin {self._stability_margin:.2f} below threshold '
                        f'{STABILITY_HALT_THRESHOLD} -> HALT'
                    )
                if slip_unsafe:
                    self.get_logger().warn(
                        f'Wheel slip {self._wheel_slip:.2f} above threshold '
                        f'{WHEEL_SLIP_HALT_THRESHOLD} -> HALT'
                    )
                if stall_unsafe:
                    self.get_logger().warn(
                        'Motion stall confirmed (LiDAR scan-consistency, motion_stall_detector_node.py) '
                        '-> HALT'
                    )
            self.state = 'HALT'
            return

        on_ramp = abs(pitch_deg) > PITCH_ON_RAMP_DEG
        transitioning = abs(pitch_rate) > TRANSITION_WINDOW_DEG

        # Phase 4: perception can pre-empt this edge before pitch moves at
        # all. Confidence-gated so a noisy/low-confidence estimate (e.g.
        # too few valid LiDAR returns, or a scattered fit) can't fire it -
        # the IMU-based condition on the right of `or` is the fallback for
        # exactly that case, and is what phase 3 already had.
        breakpoint_imminent = (
            self._percept_confidence >= PERCEPTION_MIN_CONFIDENCE
            and 0.0 <= self._percept_breakpoint_distance <= BREAKPOINT_TRIGGER_DISTANCE_M
        )

        if self.state == 'APPROACH' and on_ramp:
            self.state = 'ASCENT'
        elif self.state == 'ASCENT' and (breakpoint_imminent or (transitioning and pitch_deg < PITCH_ON_RAMP_DEG)):
            self.state = 'TRANSITION'
        elif self.state == 'ASCENT' and not on_ramp:
            # Phase 7 bug fix (found live during phase 7's own descent-
            # braking testing): ASCENT
            # was the only state in this FSM with no GUARANTEED way out.
            # Every other state has an edge keyed purely on on_ramp (EXIT,
            # DESCENT); ASCENT's only edge to TRANSITION is gated on
            # breakpoint_imminent OR a sharp pitch-RATE spike, and BOTH can
            # legitimately fail to fire at once: gentle, well-controlled
            # driving never produces the rate spike (that's the reported
            # bug), and separately - confirmed live, NOT assumed - a
            # low-friction ascent can leave ramp_classifier_node.py's
            # breakpoint_distance_m estimate stuck at a stale, non-
            # -decreasing value even with confidence comfortably above
            # PERCEPTION_MIN_CONFIDENCE (0.85-0.93 observed, not a low-
            # -confidence rejection) - so breakpoint_imminent's distance
            # gate never passes either. Investigated and ruled out: the OR
            # logic here is wired correctly (this elif chain IS reached
            # and IS evaluated every IMU callback), and it is not a sensor-
            # noise/attitude-jitter artifact - a direct high-precision
            # probe of /imu during an active low-friction climb showed
            # EXACTLY zero pitch variance (rock solid, not jittering) at
            # the same time the perception estimate was stuck, so the
            # cause is in ramp_classifier_node.py's geometric estimate
            # itself, not physical vibration feeding it bad data. Root-
            # -causing that estimator fully is future work (see README);
            # this edge is the actual fix, not a band-aid on
            # TRANSITION_WINDOW_DEG - it guarantees ASCENT can never
            # strand the FSM once the robot has genuinely left the ramp,
            # regardless of whether either upstream detector ever fires.
            # Skips the brief TRANSITION state (which exists to mark
            # "actively crossing a break point", not applicable if the
            # crossing already silently finished) straight to EXIT,
            # mirroring DESCENT's own unconditional "not on_ramp -> EXIT"
            # edge below.
            self.get_logger().warn(
                'ASCENT -> EXIT via fallback (neither perception pre-emption nor a pitch-rate '
                'spike caught this edge) - see state_machine_node.py for why this path exists'
            )
            self.state = 'EXIT'
        elif self.state == 'TRANSITION' and not on_ramp:
            self.state = 'EXIT'
        elif self.state == 'EXIT' and on_ramp:
            # re-entering a ramp downward from a flat top zone
            self.state = 'DESCENT'
        elif self.state == 'DESCENT' and not on_ramp:
            self.state = 'EXIT'
        elif self.state == 'HALT' and not margin_unsafe and not slip_unsafe and not stall_unsafe:
            # HALT-exit edge: recover to whatever state HALT actually
            # interrupted, not unconditionally to APPROACH.
            #
            # Phase 7 first special-cased just DESCENT for this (gravity
            # ASSISTS the failure mode on the way down instead of opposing
            # it the way it does climbing, so a window recovering through
            # APPROACH's much faster flat_mode profile while still
            # physically on a downhill slope is a real re-trigger risk,
            # not just a latency inconvenience). A later phase-7 pass then
            # found the same class of bug from a different direction,
            # live: a HALT blip right at the ramp crest (common in
            # low-friction settling) could interrupt EXIT, and
            # unconditionally recovering to APPROACH silently "forgot" the
            # robot had already reached EXIT - so heading downhill
            # afterward matched APPROACH's on_ramp edge (-> ASCENT)
            # instead of EXIT's (-> DESCENT), permanently misrouting that
            # pass and never activating DESCENT or descent_brake_node.py's
            # braking loop at all.
            #
            # The general fix: always recover to self._pre_halt_state
            # exactly. This subsumes the DESCENT special case (recovering
            # to DESCENT is just this rule applied to a DESCENT interrupt)
            # and fixes the EXIT case the same way, with no per-state
            # special-casing left to get wrong the next time this surfaces
            # for some other state. It's also a strict improvement for
            # ASCENT/TRANSITION, which had the same latent (if less
            # severe, since gravity doesn't assist climbing failures)
            # risk of a HALT recovering to APPROACH's faster flat_mode
            # profile while still physically on the ramp mid-climb -
            # recovering to the actual interrupted state keeps
            # ramp_mode/transition_mode's more conservative profile live
            # instead. See self._pre_halt_state's docstring in __init__
            # for why a single remembered value (not a stack) is
            # sufficient even if HALT fires again shortly after recovery.
            self.state = self._pre_halt_state

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))
        self.speed_scale_pub.publish(Float32(data=SPEED_SCALE[self.state]))


def main(args=None):
    rclpy.init(args=args)
    node = RampStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
