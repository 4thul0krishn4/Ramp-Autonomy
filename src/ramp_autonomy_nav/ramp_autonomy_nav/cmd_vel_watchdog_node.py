#!/usr/bin/env python3
"""
cmd_vel_watchdog_node.py  [PHASE 8]

Closes a real gap flagged since phase 6: nav2_param_switcher_node.py's
HALT enforcement is retry + logging + accurate state tracking, not a
redundant INDEPENDENT enforcement path - if `/controller_server/
set_parameters` silently failed or lagged, controller_server could keep
running a faster-than-safe MPPI profile for one retry-interval window
before that's caught (see that node's own docstring). This node closes
that window: while `/ramp_state == HALT`, it forces the robot's actual
outgoing velocity command toward zero regardless of what profile MPPI
is running or whether the parameter push has landed - it never touches
`/controller_server/set_parameters` or any Nav2 parameter at all, so it
cannot be affected by whatever is wrong with that path.

===================== How this avoids racing another publisher =====================
An earlier session considered - and rejected - publishing directly to
the robot's actuation topic (`/model/ramp_robot/cmd_vel`) from a new
node: that would make it a THIRD publisher on a topic controller_server
and velocity_smoother already both touch (indirectly, via remaps), with
no arbitration between them - whichever message happens to land last
wins until the next one, a genuine, undefined race (see
descent_brake_node.py's own docstring for the same reasoning applied to
a different topic).

This node avoids that by being inserted INLINE in the existing pipeline
as a single mandatory gate, not as a second parallel writer:

    controller_server -> cmd_vel_nav -> velocity_smoother
        -> cmd_vel_gated_in -> [THIS NODE] -> /model/ramp_robot/cmd_vel

velocity_smoother's output (fixed topic name `cmd_vel_smoothed`) is
remapped to `cmd_vel_gated_in` instead of straight to the robot (see
ramp_autonomy_bringup.launch.py) - this node is now the ONLY thing that
ever publishes `/model/ramp_robot/cmd_vel`. There is exactly one writer
on the real actuation topic at all times, so there is nothing left to
race: not "two nodes, arbitrated somehow" but "one node, structurally".

===================== What it actually does, and why it RAMPS instead of teleporting to zero =====================
Every `cmd_vel_gated_in` message (velocity_smoother publishes
continuously at its own smoothing_frequency, ~20 Hz, regardless of
whether new commands are arriving - see nav2_params.yaml) is forwarded
UNCHANGED while `/ramp_state != HALT`.

The FIRST version of this node published a hard Twist() (instant zero)
the moment HALT was declared, every tick, for the whole HALT duration.
Live testing (repeated full ascend/crest-cross trials, see README)
found that version measurably increased HALT-thrash (rapid repeated
HALT<->recovery cycling) compared to no watchdog at all in a small but
real fraction of runs, where the old direct pipeline never showed it.
The leading, evidence-informed explanation: teleporting the commanded
velocity to exactly zero in one tick is an effectively-infinite
deceleration spike, physically harsher than anything MPPI/
velocity_smoother would ever command on their own (both respect real
accel/decel limits) - on a marginal-friction surface, that spike is
itself plausibly enough to trigger a fresh wheel-slip or margin event,
right as the robot is trying to recover from the last one. The old
retry-based path never had this problem because nav2_param_switcher_
node.py's halt_mode push lets MPPI decelerate smoothly toward zero
within its own configured limits - this node's insta-zero was, in that
one respect, actually WORSE than the gap it was built to close.

Fixed by ramping the published command toward zero over time instead of
snapping to it - at MAX_DECEL_MPS2 / MAX_DECEL_RAD_S2, which mirror
robot.urdf.xacro's own DiffDrive plugin rating (max_linear/angular_
acceleration) exactly, not a slower, invented "gentle" limit: this
still stops the robot as fast as the physical platform is rated to
stop, just not faster-than-physical via an instant discontinuity. At a
typical 0.25-0.8 m/s pre-HALT speed this still reaches zero in roughly
0.13-0.4 s - a small, bounded, and still drastically faster/more
assured window than depending on a parameter-service retry loop that
might never succeed at all (see live-test numbers in the README).

Additionally, `/ramp_state` transitioning INTO HALT starts the ramp
immediately on that message, rather than waiting for the next
`cmd_vel_gated_in` tick to arrive (up to one smoothing period late,
~50 ms) - this node's whole point is closing timing gaps, so it closes
this one too. While ramping (and once fully at zero), incoming
`cmd_vel_gated_in` messages are NOT forwarded - the ramp/zero timer is
the sole source of truth for the actuation topic for the HALT
duration, so a stale, faster-than-safe upstream command can never leak
through mid-ramp.

This node deliberately does NOT re-derive its own HALT condition from
`/stability_margin`/`/wheel_slip` directly - it trusts `/ramp_state` for
that. That's a considered choice, not laziness: state_machine_node.py's
HALT computation was independently confirmed (phase 8's stability-margin
sampling investigation - see the README) to be synchronous with the
20 Hz IMU callback, with no slow-poll staleness of its own, so
re-deriving the same threshold logic here a second time would only add
duplicated, driftable logic without closing a real gap - the actual gap
this node closes is specifically the parameter-service path (MPPI
possibly still running a stale, faster profile), not state detection.

===================== A known, pre-existing limit this does NOT newly solve =====================
If this node itself stops running, `cmd_vel_gated_in` messages
accumulate unread and nothing publishes `/model/ramp_robot/cmd_vel` at
all - the robot holds whatever command DiffDrive last received rather
than actively stopping. That's a general single-point-of-failure risk
any inline gate has (velocity_smoother itself already carries the same
risk today, un-changed by this node), not something this specific fix
promises to solve - a real answer would need a heartbeat/liveness
pattern, out of scope here.

Published:
    /model/ramp_robot/cmd_vel   (geometry_msgs/Twist)  gated actuation
                                 command - see GZ_CMD_VEL_TOPIC in
                                 ramp_world.launch.py for why this exact
                                 topic name is not configurable.

Subscribed:
    /ramp_state          (std_msgs/String)    current FSM state
    cmd_vel_gated_in     (geometry_msgs/Twist) velocity_smoother's
                          output, remapped here instead of straight to
                          the robot - see ramp_autonomy_bringup.launch.py
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

ACTUATION_TOPIC = '/model/ramp_robot/cmd_vel'

# Match robot.urdf.xacro's DiffDrive plugin rating exactly (max_linear_
# acceleration / max_angular_acceleration) - the platform's own rated
# max safe deceleration, not an invented slower "gentle" limit. See the
# module docstring for why ramping at this rate (instead of an instant
# teleport to zero) is what fixed the HALT-thrash this node's first
# version caused.
MAX_DECEL_MPS2 = 2.0
MAX_DECEL_RAD_S2 = 3.0
RAMP_PERIOD_SEC = 0.02  # 50 Hz - fast enough that even a 0.8 m/s ceiling
                        # (flat_mode) reaches zero in ~20 ramp ticks


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self._state = 'APPROACH'
        self._halt_count = 0
        self._last_cmd = Twist()          # most recent pass-through command (pre-HALT)
        self._ramp_cmd = Twist()          # current point along the HALT deceleration ramp
        self._ramp_timer = None

        self._cmd_pub = self.create_publisher(Twist, ACTUATION_TOPIC, 10)

        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Twist, 'cmd_vel_gated_in', self._on_cmd, 10)

        self.get_logger().info(
            f'cmd_vel watchdog running: gating cmd_vel_gated_in -> {ACTUATION_TOPIC} on /ramp_state HALT '
            f'(ramped stop at {MAX_DECEL_MPS2} m/s^2 / {MAX_DECEL_RAD_S2} rad/s^2, matching the platform\'s '
            'own rated deceleration), independent of controller_server/set_parameters'
        )

    def _on_state(self, msg: String):
        was_halt = self._state == 'HALT'
        self._state = msg.data
        if self._state == 'HALT' and not was_halt:
            # HALT-entry edge: start the deceleration ramp immediately
            # (don't wait for the next pass-through tick, up to one
            # smoothing period late) from whatever was last actually
            # commanded - see module docstring for why this ramps
            # instead of teleporting straight to zero.
            self._halt_count += 1
            self.get_logger().warn(
                f'HALT #{self._halt_count}: ramping cmd_vel to zero at the platform\'s rated deceleration, '
                'independent of MPPI profile / set_parameters state'
            )
            self._ramp_cmd = Twist(linear=self._last_cmd.linear, angular=self._last_cmd.angular)
            if self._ramp_timer is not None:
                self._ramp_timer.cancel()
            self._ramp_timer = self.create_timer(RAMP_PERIOD_SEC, self._on_ramp_tick)
        elif self._state != 'HALT' and was_halt:
            # HALT-exit edge: stop ramping, resume pure pass-through.
            if self._ramp_timer is not None:
                self._ramp_timer.cancel()
                self._ramp_timer = None

    def _on_ramp_tick(self):
        step_lin = MAX_DECEL_MPS2 * RAMP_PERIOD_SEC
        step_ang = MAX_DECEL_RAD_S2 * RAMP_PERIOD_SEC
        self._ramp_cmd.linear.x = _step_toward_zero(self._ramp_cmd.linear.x, step_lin)
        self._ramp_cmd.linear.y = _step_toward_zero(self._ramp_cmd.linear.y, step_lin)
        self._ramp_cmd.angular.z = _step_toward_zero(self._ramp_cmd.angular.z, step_ang)
        self._cmd_pub.publish(self._ramp_cmd)
        if self._ramp_cmd.linear.x == 0.0 and self._ramp_cmd.linear.y == 0.0 and self._ramp_cmd.angular.z == 0.0:
            # Fully stopped - collapse to a plain, cheap keepalive instead
            # of continuing to recompute a no-op ramp every tick.
            self._ramp_timer.cancel()
            self._ramp_timer = self.create_timer(RAMP_PERIOD_SEC, lambda: self._cmd_pub.publish(Twist()))

    def _on_cmd(self, msg: Twist):
        if self._state == 'HALT':
            return  # ramp timer (started on HALT-entry) is the sole publisher during HALT
        self._last_cmd = msg
        self._cmd_pub.publish(msg)  # pass-through, unchanged


def _step_toward_zero(value: float, step: float) -> float:
    if value > 0.0:
        return max(0.0, value - step)
    if value < 0.0:
        return min(0.0, value + step)
    return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
