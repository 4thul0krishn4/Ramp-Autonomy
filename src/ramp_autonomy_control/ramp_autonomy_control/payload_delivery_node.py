#!/usr/bin/env python3
"""
payload_delivery_node.py  [delivery session]

Watches the robot's own EKF pose for "stopped, inside the Floor 2 delivery pad, still carrying the payload" and, when that holds for DWELL_S, triggers the release:
publishes /payload/detach (the gz DetachableJoint's own detach topic, bridged from ROS - see ramp_world.launch.py) and, once gz confirms `detached` on /payload/state,
zeroes stability_monitor_node.py's payload_mass_kg parameter so the tip-over margin (and, if it is ever plumbed in, the descent brake) reflect the robot's now-real
unloaded mass rather than the launch-time value - the physics itself is already correct the instant the DetachableJoint releases (the payload leaves with its own mass),
this only fixes a ROS-side parameter that would otherwise still say "loaded" until the next full launch.

This node does NOT decide where the robot goes or when it stops - the mission's own navigation does that, exactly as everywhere else in this project (the state machine
estimates state, nav2_param_switcher applies it, this node fires an event) - it only recognizes "we are here, stopped, with cargo" and fires the one-shot release.
A stationary but UNLOADED robot standing in the same zone does nothing (nothing to release); a LOADED robot passing through without stopping does nothing (dwell timer
resets on any motion) - this is not a docking controller, it does not steer the robot onto the pad.

Zone (odom frame; odom x = world x - 1.0, per every other test driver in this project): the Floor 2 delivery pad is world x 78.0-81.0, y +-1.5
(floor2_delivery_zone, generate_floor2.py) -> odom x 77.0-80.0, y +-1.5. Slightly INSET from the physical pad (ZONE_MARGIN_M) so the trigger fires only once the robot
is solidly on the pad, not the instant its footprint first overlaps the edge.

Published:
    /payload/detach     (std_msgs/Empty)   one-shot release trigger (bridged to gz's own /payload/detach - see the module docstring above)
    /delivery/status     (std_msgs/String)  human-readable state for logging/diagnostics: WAITING / DWELLING / RELEASED / UNLOADED

Subscribed:
    /odometry/filtered   (nav_msgs/Odometry)   EKF pose + body twist (same source every other test/driver in this project treats as ground truth for position)
    /payload/state        (std_msgs/String)     gz DetachableJoint's own attach/detach confirmation (bridged - see ramp_world.launch.py)

Calls:
    /stability_monitor/set_parameters   (rcl_interfaces/srv/SetParameters)   zero payload_mass_kg once gz confirms the release (see docstring above)
"""
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Empty, String

# Defaults assume the standard mission (robot spawns at the gate, gz x=1.0 -> odom (0,0,0), so odom x = gz x - 1.0 throughout, the convention every driver in this project
# uses). Declared as ROS parameters (not just module constants) so a test that spawns the robot somewhere else (e.g. directly on the pad, to smoke-test this node in
# isolation) can override them - see validation/phase8/delivery/trigger_smoke.py.
ZONE_X_MIN_DEFAULT, ZONE_X_MAX_DEFAULT = 77.3, 79.7   # odom frame; 0.3 m inset from the 77.0-80.0 physical pad on the approach side (ramp-2 exit), full inset on the far side (a wall is there anyway)
ZONE_Y_MAX_DEFAULT = 1.2                                # inset from +-1.5 m (also keeps the robot clear of the delivery dock at y >= 0.44)
STOPPED_MPS = 0.03                      # body-frame forward speed below this counts as "stopped" (matches the stationary_v0 threshold this project's other dwell checks use)
DWELL_S = 1.0                           # must be inside the zone AND stopped for this long before releasing - see module docstring (not a docking controller: no motion here)


class PayloadDelivery(Node):
    def __init__(self):
        super().__init__('payload_delivery')
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._attached = True           # optimistic default (matches every other node's "assume the validated/safe state until told otherwise" convention in this project)
        self._released = False          # one-shot latch: never re-trigger even if the robot re-enters the zone later (e.g. driving back through on the return leg)
        self._dwell_start = None
        self._mass_zeroed = False

        self.declare_parameter('zone_x_min', ZONE_X_MIN_DEFAULT)
        self.declare_parameter('zone_x_max', ZONE_X_MAX_DEFAULT)
        self.declare_parameter('zone_y_max', ZONE_Y_MAX_DEFAULT)
        self._zone_x_min = self.get_parameter('zone_x_min').value
        self._zone_x_max = self.get_parameter('zone_x_max').value
        self._zone_y_max = self.get_parameter('zone_y_max').value
        self.add_on_set_parameters_callback(self._on_set_zone_params)

        self.detach_pub = self.create_publisher(Empty, '/payload/detach', 10)
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 20)
        self.create_subscription(String, '/payload/state', self._on_payload_state, 10)
        self._set_params_client = self.create_client(SetParameters, '/stability_monitor/set_parameters')

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'Payload delivery watcher running: zone odom x [{self._zone_x_min}, {self._zone_x_max}] y +-{self._zone_y_max}, dwell {DWELL_S} s'
        )

    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._vx = msg.twist.twist.linear.x

    def _on_payload_state(self, msg: String):
        was_attached = self._attached
        self._attached = msg.data != 'detached'
        if was_attached and not self._attached:
            self.get_logger().info('gz confirms the payload detached - zeroing stability_monitor payload_mass_kg')
            self._zero_payload_mass()

    def _in_zone(self) -> bool:
        return self._zone_x_min <= self._x <= self._zone_x_max and abs(self._y) <= self._zone_y_max

    def _tick(self):
        status = 'UNLOADED'
        if self._attached and not self._released:
            in_zone = self._in_zone()
            stopped = abs(self._vx) < STOPPED_MPS
            if in_zone and stopped:
                if self._dwell_start is None:
                    self._dwell_start = self.get_clock().now()
                dwell = (self.get_clock().now() - self._dwell_start).nanoseconds * 1e-9
                status = 'DWELLING'
                if dwell >= DWELL_S:
                    self.get_logger().info(f'in the delivery zone, stopped for {dwell:.1f} s - releasing the payload')
                    self.detach_pub.publish(Empty())
                    self._released = True
                    status = 'RELEASED'
            else:
                self._dwell_start = None
                status = 'WAITING'
        elif self._released:
            status = 'RELEASED'
        self.status_pub.publish(String(data=status))

    def _on_set_zone_params(self, params):
        for p in params:
            if p.name == 'zone_x_min': self._zone_x_min = p.value
            elif p.name == 'zone_x_max': self._zone_x_max = p.value
            elif p.name == 'zone_y_max': self._zone_y_max = p.value
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _zero_payload_mass(self):
        if self._mass_zeroed or not self._set_params_client.service_is_ready():
            if not self._mass_zeroed:
                self.get_logger().warn('/stability_monitor/set_parameters not ready yet - payload_mass_kg will stay at its launch value until the next tick')
            return
        req = SetParameters.Request(parameters=[
            Parameter(name='payload_mass_kg', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.0)),
        ])
        self._mass_zeroed = True
        future = self._set_params_client.call_async(req)
        future.add_done_callback(self._on_zero_response)

    def _on_zero_response(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'zeroing payload_mass_kg failed: {exc}')
            self._mass_zeroed = False
            return
        failed = [r.reason for r in resp.results if not r.successful]
        if failed:
            self.get_logger().error(f'stability_monitor rejected payload_mass_kg=0.0: {failed}')
            self._mass_zeroed = False
        else:
            self.get_logger().info('stability_monitor payload_mass_kg confirmed 0.0')

    def _retry_zero_if_needed(self):
        if not self._mass_zeroed and not self._attached:
            self._zero_payload_mass()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadDelivery()

    def retry_tick():
        node._retry_zero_if_needed()
    node.create_timer(1.0, retry_tick)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
