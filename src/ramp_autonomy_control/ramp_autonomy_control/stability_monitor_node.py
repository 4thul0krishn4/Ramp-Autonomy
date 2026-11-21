#!/usr/bin/env python3
"""
stability_monitor_node.py  [PHASE 6]

Computes two safety-critical scalars in real time and publishes them
for state_machine_node.py to act on:

    /stability_margin  (std_msgs/Float32) - normalized tip-over margin,
        1.0 = fully stable / flat ground, 0.0 = at the tipping edge.
        Derived from IMU pitch/roll + payload mass/CG offset (params
        below) vs. the robot's support-polygon geometry. THIS IS A
        SIMPLIFIED PLANAR MODEL (pitch-axis only) - a full 3D
        support-polygon calculation accounting for roll simultaneously
        is a documented next step, not implemented here yet.

    /wheel_slip         (std_msgs/Float32) - max slip ratio across the
        4 wheels, comparing commanded per-wheel angular velocity
        (derived from /model/ramp_robot/cmd_vel via the same diff-drive
        kinematics the DiffDrive actuation plugin itself uses) against
        actual per-wheel angular velocity (/joint_states, from
        gz-sim-joint-state-publisher-system - real physics ground
        truth, independent of what's doing the actuating).

===================== Architecture decision (phase 6) =====================
This does NOT require migrating off gz-sim-diff-drive-system to
ros2_control. DiffDrive only exposes 2 commanded speeds (left/right
side - see robot.urdf.xacro's <left_joint>/<right_joint> pairing), but
it still actuates all 4 wheel joints independently under the hood, and
the physics engine simulates each of those 4 joints' actual velocity
separately - they can and do differ under load/slip even though 2 of
them share a commanded value. gz-sim-joint-state-publisher-system is a
read-only ground-truth reporter that doesn't care what plugin is doing
the actuating; it's the missing "actual" half of the slip comparison,
added as a second, independent Gazebo system plugin rather than by
replacing DiffDrive. A migration to ros2_control (new controller
manager, hardware interfaces, 4 independent command interfaces) would
be the right call if something later needs independent PER-WHEEL
torque/velocity COMMANDS - slip ESTIMATION never needed that, only
per-wheel READBACK, which the lighter path above already provides.

Params (declare at launch, override per test run - see phase 8's test
matrix, which should sweep these):
    payload_mass_kg        (float, default 0.0)
    payload_cg_offset_m    (float, default 0.0)   forward(+)/back(-)
    wheelbase_m            (float, default 0.56)  front-to-back wheel spacing
    track_width_m          (float, default 0.7)   left-to-right wheel spacing
    base_mass_kg            (float, default 40.0)
    base_height_m           (float, default 0.25)
    wheel_radius_m          (float, default 0.12)  must match robot.urdf.xacro
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

# Which side of the robot each wheel joint is on, for deriving its
# commanded angular velocity from the body twist (matches DiffDrive's own
# <left_joint>/<right_joint> grouping in robot.urdf.xacro).
WHEEL_JOINT_SIDE = {
    'front_left_wheel_joint': 'left',
    'rear_left_wheel_joint': 'left',
    'front_right_wheel_joint': 'right',
    'rear_right_wheel_joint': 'right',
}

# Below this commanded wheel speed (rad/s), don't report slip for that
# wheel - a near-stationary robot with a tiny commanded speed makes the
# slip ratio's denominator blow up into a meaningless huge number from
# ordinary sensor noise, not real loss of traction.
MIN_COMMANDED_SPEED_RAD_S = 0.05

# Topic the DiffDrive plugin actually reads (see ramp_world.launch.py's
# GZ_CMD_VEL_TOPIC comment for why this isn't a bare /cmd_vel). This is
# the commanded signal actually reaching the wheels right now - whatever
# sits upstream (Nav2's velocity_smoother, a manual test publish, etc.)
# doesn't matter, this is what's real to compare actual velocity against.
CMD_VEL_TOPIC = '/model/ramp_robot/cmd_vel'


class StabilityMonitor(Node):
    def __init__(self):
        super().__init__('stability_monitor')

        self.declare_parameter('payload_mass_kg', 0.0)
        self.declare_parameter('payload_cg_offset_m', 0.0)
        self.declare_parameter('wheelbase_m', 0.56)
        self.declare_parameter('track_width_m', 0.7)
        self.declare_parameter('base_mass_kg', 40.0)
        self.declare_parameter('base_height_m', 0.25)
        self.declare_parameter('wheel_radius_m', 0.12)

        self._last_cmd = Twist()  # optimistic default: not moving

        self.margin_pub = self.create_publisher(Float32, '/stability_margin', 10)
        self.slip_pub = self.create_publisher(Float32, '/wheel_slip', 10)

        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 20)

        self.get_logger().info(
            'Stability monitor running (planar pitch-axis margin model + per-wheel slip)'
        )

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)

        margin = self._compute_pitch_margin(pitch_rad)
        self.margin_pub.publish(Float32(data=margin))

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = msg

    def _on_joint_states(self, msg: JointState):
        slip = self._compute_wheel_slip(msg)
        self.slip_pub.publish(Float32(data=slip))

    def _compute_pitch_margin(self, pitch_rad: float) -> float:
        """
        Simplified planar tip-over margin about the pitch axis.

        Model: combined CG (base + payload) projected onto the ground
        plane must stay within the wheelbase footprint for the robot
        not to tip forward/backward. As pitch increases, the effective
        support polygon shrinks in the direction of the incline.

        This treats the robot as a rigid body on a rigid incline and
        ignores suspension compliance and dynamic (braking/accel)
        load transfer - both real effects that a full implementation
        should add. Good enough as a first-pass safety signal, not a
        certified stability calculation.
        """
        payload_mass = self.get_parameter('payload_mass_kg').value
        payload_cg = self.get_parameter('payload_cg_offset_m').value
        wheelbase = self.get_parameter('wheelbase_m').value
        base_mass = self.get_parameter('base_mass_kg').value
        base_height = self.get_parameter('base_height_m').value

        total_mass = base_mass + payload_mass
        if total_mass <= 0:
            return 1.0

        # Combined CG offset from base_link origin (base assumed centered at 0)
        combined_cg_x = (payload_mass * payload_cg) / total_mass

        # Effective forward support limit shrinks as pitch increases:
        # projecting the half-wheelbase onto the incline-adjusted plane.
        half_wheelbase = wheelbase / 2.0
        # Height of combined CG above ground (rough estimate)
        cg_height = base_height / 2.0 + (0.15 if payload_mass > 0 else 0.0)

        # Additional CG shift induced by pitch itself (rigid body on incline):
        # a CG at height h, on a surface pitched by theta, has its
        # effective horizontal position shifted by h * tan(theta).
        pitch_induced_shift = cg_height * math.tan(abs(pitch_rad))

        effective_cg_x = abs(combined_cg_x) + pitch_induced_shift

        margin = 1.0 - (effective_cg_x / half_wheelbase)
        return max(0.0, min(1.0, margin))

    def _compute_wheel_slip(self, joint_state: JointState) -> float:
        """
        Max slip ratio across the 4 wheels: |commanded - actual| / |commanded|,
        where commanded is this wheel's side (left/right) angular velocity
        derived from the last /model/ramp_robot/cmd_vel via the same
        differential-drive kinematics DiffDrive itself uses, and actual is
        this wheel joint's real simulated angular velocity from
        /joint_states.
        """
        track_width = self.get_parameter('track_width_m').value
        wheel_radius = self.get_parameter('wheel_radius_m').value
        half_track = track_width / 2.0

        vx = self._last_cmd.linear.x
        wz = self._last_cmd.angular.z
        commanded_side = {
            'left': (vx - wz * half_track) / wheel_radius,
            'right': (vx + wz * half_track) / wheel_radius,
        }

        max_slip = 0.0
        for name, velocity in zip(joint_state.name, joint_state.velocity):
            side = WHEEL_JOINT_SIDE.get(name)
            if side is None:
                continue  # not one of our 4 wheel joints
            cmd = commanded_side[side]
            if abs(cmd) < MIN_COMMANDED_SPEED_RAD_S:
                continue  # not really commanded to move - nothing meaningful to compare
            slip = abs(cmd - velocity) / abs(cmd)
            max_slip = max(max_slip, slip)
        return max_slip


def main(args=None):
    rclpy.init(args=args)
    node = StabilityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
