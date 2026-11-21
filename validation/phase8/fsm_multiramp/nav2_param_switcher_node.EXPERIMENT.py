#!/usr/bin/env python3
"""
nav2_param_switcher_node.py

Phase 3: bridges the ramp state machine to Nav2's MPPI controller.
Subscribes to /ramp_state (as published by state_machine_node.py) and,
on every state change, pushes the matching velocity/heading-tolerance
profile from ramp_mppi_params.yaml into controller_server's live MPPI
parameters via its standard rcl_interfaces /set_parameters service -
the same dynamic-reconfiguration mechanism `ros2 param set` uses, just
called from code instead of the CLI.

Why a separate node instead of extending state_machine_node.py:
  - state_machine_node.py's job is state ESTIMATION (IMU pitch in,
    /ramp_state + /cmd_vel_scale out) - it has no knowledge of Nav2 and
    shouldn't need any. Bolting an async parameter-service client with
    its own retry/backoff logic onto it would mix two concerns that
    have nothing to do with each other and are likely to evolve on
    different schedules (phase 4 will replace the FSM's pitch heuristic
    entirely; the Nav2 wiring here won't need to change when that
    happens).
  - Keeping the Nav2 integration in its own node means it can be killed,
    restarted, or swapped out (e.g. once phase 5 wants a more elaborate
    per-state cost profile than three fixed presets) without touching
    the already-verified FSM code, and it mirrors how the rest of this
    project already splits concerns into single-purpose nodes.

===================== Phase 6: failure handling =====================
Flagged gap from the phase 3 session: what happens if the
set_parameters call fails, or controller_server isn't active yet? The
original version optimistically marked a profile as "applied" the
moment it SENT the request, only reverting on an explicit failure
response - meaning a request that never got a response at all (e.g.
the service wasn't ready, so nothing was ever sent) left the node
believing whatever the PREVIOUS profile was was still correctly
applied, with no retry beyond hoping the next /ramp_state tick's
early-return guard didn't swallow it. Concretely: if a controller_server
crash-and-respawn reset it to its params-file defaults while /ramp_state
stayed unchanged (no state transition to re-trigger a push), the old
code would never notice or re-push anything.

Fixed by separating two beliefs that were previously conflated into one
variable:
  - self._target_profile: what /ramp_state currently says SHOULD be applied.
  - self._confirmed_profile: what we have an explicit SUCCESS response for.
A push is retried on a fast timer (RETRY_INTERVAL_SEC) any time these two
differ - not just reactively on the next state change - and even once
they match, a slow periodic keepalive re-push (KEEPALIVE_EVERY_N_TICKS)
guards against exactly the silent-reset scenario above. Every gap
between target and confirmed is logged at ERROR, and the log message
explicitly says whether the controller might still be running something
FASTER than intended (the actually unsafe direction) or merely hasn't
caught up to a newly-safer target yet (not a safety risk, just latency).

This is still retry + logging, not a redundant INDEPENDENT enforcement
path - during that unsafe-direction gap, the actual velocity commands
this node cannot influence any faster than RETRY_INTERVAL_SEC. Phase 8
closes that specific window with a second, genuinely independent
mechanism that doesn't touch set_parameters at all:
cmd_vel_watchdog_node.py ramps the robot's real outgoing command to zero
whenever `/ramp_state == HALT`, regardless of what MPPI profile is
actually still running - see that node's own docstring for the full
design (why it's inserted as a single mandatory pipeline gate rather
than a second publisher racing this reconfiguration path).

===================== Phase 7/8: descent_brake_node.py's parameter domain =====================
Phase 7 first had descent_brake_node.py adjust FollowPath.vx_max during
DESCENT, which meant this node's periodic keepalive re-push had to be
specially suppressed during DESCENT (it would have clobbered the other
node's live-adjusted vx_max back to the static ceiling). A later
phase-7 pass switched descent_brake_node.py to adjust FollowPath.ax_max/az_max
instead (see its own docstring for why - live testing showed vx_max
-capping cost more distance than it saved in slip). This node and
descent_brake_node.py now touch entirely disjoint parameter sets
(vx_max/vx_min/wz_max/heading-tolerance here, ax_max/ax_min/az_max
there), so the DESCENT-specific keepalive suppression is no longer
needed and has been removed - this node now treats every state
uniformly, including DESCENT, which still gets ramp_mode's normal
vx_max/wz_max exactly as before.

Published:
    (none)

Subscribed:
    /ramp_state   (std_msgs/String)  current FSM state

Calls:
    /controller_server/set_parameters  (rcl_interfaces/srv/SetParameters)
"""
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String

# Which ramp_mppi_params.yaml profile applies in each FSM state.
# HALT maps to halt_mode (max_vel_x/max_vel_theta = 0.0) rather than
# reusing ramp_mode, so a stability halt is enforced by the planner
# itself and doesn't rely solely on /cmd_vel_scale having a consumer.
#
# ASCENT and DESCENT split into separate profiles (both used to share
# ramp_mode) as part of the cross-track-error fix: two live-tested
# critic-tuning attempts on PathAlignCritic alone (weight 14->60 at
# power=1, then weight=8/power=2) both plateaued around 2.4-2.8 m of
# cross-track error without ever correcting back toward center - same
# qualitative failure regardless of weight/power (see
# validation/phase8/crosstrack_diagnosis/). Root cause: a DiffDrive
# robot can only correct laterally by turning, and ramp_mode's
# heading_tolerance_deg=5.0 (pushed to PathAngleCritic.
# max_angle_to_furthest) was actively penalizing the very heading
# deviation MPPI would need to adopt to steer back to center - fighting
# PathAlignCritic regardless of how strongly PathAlignCritic itself was
# weighted. ASCENT has already been proven driftless at the tight 5 deg
# tolerance (every ascent test this project has run), so rather than
# loosen it broadly and risk regressing that, DESCENT now gets its own
# profile with a looser tolerance - ascent's proven behavior is
# unaffected.
RAMP_STATE_PROFILE = {
    'APPROACH': 'flat_mode',
    'ASCENT': 'ramp_mode',
    'TRANSITION': 'transition_mode',
    'DESCENT': 'descent_mode',
    'EXIT': 'flat_mode',
    'HALT': 'halt_mode',
}

CONTROLLER_PLUGIN = 'FollowPath'  # controller_plugins entry in nav2_params.yaml

# Fast retry while target and confirmed profile disagree - this is the
# safety-critical gap, kept short (comparable to the FSM's own 0.1 s
# publish period) so the worst case window a wrong-speed profile could
# still be live is small and bounded, not "whenever the next state
# transition happens to occur".
RETRY_INTERVAL_SEC = 0.2
# Once confirmed == target, re-push anyway every this-many retry ticks
# as a slow keepalive, in case controller_server silently reset (crash +
# respawn) without a /ramp_state change to notice it by.
import os as _os
KEEPALIVE_EVERY_N_TICKS = int(_os.environ.get('SWITCHER_KEEPALIVE_N', '15'))  # EXPERIMENT ONLY (keepalive isolation): default 15 = ~3 s at RETRY_INTERVAL_SEC=0.2


def _double_param(name: str, value: float) -> Parameter:
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
    )


def _int_param(name: str, value: int) -> Parameter:
    # PathAlignCritic's cost_power is declared as `unsigned int power_` in
    # nav2_mppi_controller's own source (path_align_critic.hpp) - must be
    # pushed as PARAMETER_INTEGER, not PARAMETER_DOUBLE like everything
    # else here, or set_parameters rejects it (wrong type for the
    # declared parameter).
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value)),
    )


class Nav2ParamSwitcher(Node):
    def __init__(self):
        super().__init__('nav2_param_switcher')

        params_path = Path(get_package_share_directory('ramp_autonomy_nav')) / 'config' / 'ramp_mppi_params.yaml'
        with open(params_path) as f:
            self._profiles = yaml.safe_load(f)

        missing = [p for p in RAMP_STATE_PROFILE.values() if p not in self._profiles]
        if missing:
            raise RuntimeError(
                f'ramp_mppi_params.yaml is missing profile(s) {missing} referenced by '
                f'RAMP_STATE_PROFILE - state and param file have drifted apart.'
            )

        self._target_profile = RAMP_STATE_PROFILE['APPROACH']  # robot always starts in APPROACH
        self._confirmed_profile = None  # nothing confirmed applied yet - forces an initial push
        self._push_in_flight = False
        self._keepalive_tick = 0

        self._set_params_client = self.create_client(SetParameters, '/controller_server/set_parameters')

        self.create_subscription(String, '/ramp_state', self._on_ramp_state, 10)
        self.create_timer(RETRY_INTERVAL_SEC, self._on_retry_timer)

        self.get_logger().info(
            f'Nav2 param switcher started, profiles loaded from {params_path}'
        )

    def _on_ramp_state(self, msg: String):
        profile_name = RAMP_STATE_PROFILE.get(msg.data)
        if profile_name is None:
            self.get_logger().warn(f'Unknown /ramp_state value "{msg.data}", ignoring')
            return
        if profile_name == self._target_profile:
            return  # already the target - /ramp_state republishes at 10 Hz even when unchanged
        self._target_profile = profile_name
        self._attempt_push()  # try immediately rather than waiting for the next retry tick

    def _on_retry_timer(self):
        if self._push_in_flight:
            return
        if self._target_profile != self._confirmed_profile:
            self.get_logger().error(self._gap_description())
            self._attempt_push()
            return
        # Target and confirmed agree - slow keepalive re-push in case
        # controller_server silently lost its parameters (e.g. crash +
        # respawn) without a /ramp_state change to notice by. Applies
        # uniformly to every state including DESCENT as of that later phase-7
        # pass - see
        # the module docstring for why that's now safe (disjoint
        # parameter sets with descent_brake_node.py).
        self._keepalive_tick += 1
        if self._keepalive_tick >= KEEPALIVE_EVERY_N_TICKS:
            self._keepalive_tick = 0
            self._attempt_push()

    def _gap_description(self) -> str:
        if self._confirmed_profile is None:
            return (
                f'MPPI profile UNCONFIRMED: target is "{self._target_profile}" but nothing has '
                f'been confirmed applied to controller_server yet - do not assume it is running '
                f'a safe speed limit.'
            )
        target_vx = self._vx_max(self._target_profile)
        confirmed_vx = self._vx_max(self._confirmed_profile)
        if confirmed_vx > target_vx:
            return (
                f'MPPI profile UNSAFE GAP: controller_server may still be running '
                f'"{self._confirmed_profile}" (vx_max={confirmed_vx}), which is FASTER than the '
                f'current target "{self._target_profile}" (vx_max={target_vx}). Retrying push.'
            )
        return (
            f'MPPI profile lagging (not unsafe): controller_server confirmed on '
            f'"{self._confirmed_profile}" (vx_max={confirmed_vx}), target "{self._target_profile}" '
            f'(vx_max={target_vx}) is not faster. Retrying push.'
        )

    def _vx_max(self, profile_name: str) -> float:
        return float(self._profiles[profile_name]['max_vel_x'])

    def _attempt_push(self):
        if not self._set_params_client.service_is_ready():
            # controller_server may still be configuring/activating (lifecycle_manager
            # autostart isn't instant), or may be down. Nothing to send yet - the
            # retry timer will keep calling this until it is ready.
            self.get_logger().error(
                '/controller_server/set_parameters not available - '
                + self._gap_description()
            )
            return

        target = self._target_profile
        profile = self._profiles[target]
        vx_max = float(profile['max_vel_x'])
        wz_max = float(profile['max_vel_theta'])
        heading_tolerance_rad = math.radians(float(profile['heading_tolerance_deg']))
        path_align_weight = float(profile['path_align_weight'])
        path_align_power = int(profile['path_align_power'])
        path_align_offset = int(profile['path_align_offset'])

        request = SetParameters.Request(parameters=[
            _double_param(f'{CONTROLLER_PLUGIN}.vx_max', vx_max),
            # Symmetric reverse cap derived from vx_max (not its own yaml field) so
            # halt_mode's max_vel_x=0.0 actually zeroes reverse creep too.
            _double_param(f'{CONTROLLER_PLUGIN}.vx_min', -vx_max),
            _double_param(f'{CONTROLLER_PLUGIN}.wz_max', wz_max),
            _double_param(f'{CONTROLLER_PLUGIN}.PathAngleCritic.max_angle_to_furthest', heading_tolerance_rad),
            # Cross-track-error fix: PathAlignCritic used to stay at
            # nav2_params.yaml's single static cost_weight/cost_power/
            # offset_from_furthest for every mode - confirmed live to let
            # cross-track error grow unchecked during ramp descent (0.44 m
            # -> 2.85 m, smooth and monotonic, zero restoring force - see
            # validation/phase8/crosstrack_diagnosis/). The critic's own
            # source (path_align_critic.cpp) shows it does not apply AT
            # ALL until the robot has advanced offset_from_furthest points
            # into the path - the static default (20) exceeds this
            # project's entire ~15-point descent path, so it was
            # structurally disabled the whole time regardless of weight/
            # power. All three now mode-specific - see
            # ramp_mppi_params.yaml's descent_mode for the full story.
            _double_param(f'{CONTROLLER_PLUGIN}.PathAlignCritic.cost_weight', path_align_weight),
            _int_param(f'{CONTROLLER_PLUGIN}.PathAlignCritic.cost_power', path_align_power),
            _int_param(f'{CONTROLLER_PLUGIN}.PathAlignCritic.offset_from_furthest', path_align_offset),
        ])

        self._push_in_flight = True
        future = self._set_params_client.call_async(request)
        future.add_done_callback(
            lambda f, target_when_sent=target: self._on_set_params_response(f, target_when_sent)
        )

    def _on_set_params_response(self, future, target_when_sent: str):
        self._push_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'set_parameters call raised for "{target_when_sent}": {exc} - will retry'
            )
            return

        failed = [r.reason for r in response.results if not r.successful]
        if failed:
            self.get_logger().error(
                f'set_parameters REJECTED for "{target_when_sent}": {failed} - will retry'
            )
            return

        if target_when_sent == self._target_profile:
            self._confirmed_profile = target_when_sent
            self._keepalive_tick = 0
            self.get_logger().info(f'confirmed MPPI profile "{target_when_sent}" applied')
        else:
            # Target moved on again while this call was in flight - what we just
            # confirmed is already stale. Don't mark it confirmed; the retry timer
            # will push the current target next since they now disagree.
            self.get_logger().info(
                f'set_parameters for "{target_when_sent}" succeeded, but target has since moved '
                f'to "{self._target_profile}" - pushing that next'
            )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ParamSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
