"""
ramp_autonomy_bringup.launch.py

Single entry point for the whole project: spawns the ramp world +
robot (via ramp_autonomy_gazebo), publishes robot_description, and
starts all the autonomy nodes (state machine, perception, stability
monitor, descent brake, and - phase 8 - an independent cmd_vel
watchdog sitting between velocity_smoother and the robot's real
actuation topic; see cmd_vel_watchdog_node.py).

Usage:
    ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \\
        world:=sharp payload_mass:=100.0 payload_cg_offset:=0.1

Note: perception (phase 4) and descent_brake (phase 7) are both fully
implemented now - see ramp_classifier_node.py and descent_brake_node.py.
Both were included in this launch graph from the start (back when they
were scaffolds) precisely so no launch restructuring was needed once
they were.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('ramp_autonomy_gazebo'), 'launch', 'ramp_world.launch.py'
    )
    xacro_file = os.path.join(
        get_package_share_directory('ramp_autonomy_description'), 'urdf', 'robot.urdf.xacro'
    )

    nav_pkg = get_package_share_directory('ramp_autonomy_nav')
    nav2_params_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(nav_pkg, 'config', 'ekf.yaml')

    world_arg = DeclareLaunchArgument('world', default_value='sharp')
    payload_mass_arg = DeclareLaunchArgument('payload_mass', default_value='0.0')
    payload_cg_offset_arg = DeclareLaunchArgument('payload_cg_offset', default_value='0.0')
    detachable_payload_arg = DeclareLaunchArgument('detachable_payload', default_value='false')
    cargo_mode_arg = DeclareLaunchArgument(
        'cargo_mode', default_value='secured',
        description="'secured' (default, unaffected by this arg) or 'loose' (unsecured-cargo physics DEMO, opt-in - see robot.urdf.xacro's own arg comment "
                    "and docs/construction_site_design.md 'Unsecured cargo demo mode'). Passed through to ramp_world.launch.py and the robot_description xacro."
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='1.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.23')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Passed through to ramp_world.launch.py - see its own description.'
    )

    # NOTE: Command(...) returns a substitution that launch_ros will try to
    # YAML-parse by default - for a big XML blob like robot_description that
    # blows up (e.g. on any ':' or '-' it doesn't like). Wrapping it in
    # ParameterValue(..., value_type=str) forces it to be treated as a
    # plain string parameter instead.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' payload_mass:=', LaunchConfiguration('payload_mass'),
            ' payload_cg_offset:=', LaunchConfiguration('payload_cg_offset'),
            ' detachable_payload:=', LaunchConfiguration('detachable_payload'),
            ' cargo_mode:=', LaunchConfiguration('cargo_mode'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'payload_mass': LaunchConfiguration('payload_mass'),
            'payload_cg_offset': LaunchConfiguration('payload_cg_offset'),
            'detachable_payload': LaunchConfiguration('detachable_payload'),
            'cargo_mode': LaunchConfiguration('cargo_mode'),
            'spawn_x': LaunchConfiguration('spawn_x'), 'spawn_y': LaunchConfiguration('spawn_y'), 'spawn_z': LaunchConfiguration('spawn_z'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    state_machine = Node(
        package='ramp_autonomy_nav', executable='state_machine_node', output='screen',
    )
    perception = Node(
        package='ramp_autonomy_perception', executable='ramp_classifier_node', output='screen',
    )
    stability_monitor = Node(
        package='ramp_autonomy_control', executable='stability_monitor_node', output='screen',
        parameters=[{
            'payload_mass_kg': LaunchConfiguration('payload_mass'),
            'payload_cg_offset_m': LaunchConfiguration('payload_cg_offset'),
        }],
    )
    descent_brake = Node(
        package='ramp_autonomy_control', executable='descent_brake_node', output='screen',
    )
    motion_stall_detector = Node(
        package='ramp_autonomy_control', executable='motion_stall_detector_node', output='screen',
    )

    # ===================== EKF fusion (phase 8 session A/B follow-up) =====================
    # Fuses wheel odometry (/odom) + IMU (/imu) into /odometry/filtered and
    # publishes odom -> base_footprint TF itself - see ekf.yaml for the full
    # rationale (live testing showed wheel odom's yaw diverges from ground
    # truth under any turn, not just a fast open-loop spin; the IMU's
    # gyro-based orientation isn't wheel-slip-dependent). Must be up before
    # controller_server, which now reads odom_topic: /odometry/filtered
    # (nav2_params.yaml) instead of raw /odom.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
    )

    # ===================== Phase 3: Nav2 MPPI controller =====================
    # controller_server hosts the nav2_mppi_controller plugin (FollowPath) and
    # its own internal local_costmap - see ramp_autonomy_nav/config/nav2_params.yaml
    # for why this is scoped to the controller layer only (no planner/bt_navigator/
    # amcl yet). lifecycle_manager brings it from unconfigured -> active on its own
    # (autostart: true in nav2_params.yaml) so it's ready without a manual service
    # call. nav2_param_switcher_node then keeps its MPPI velocity/heading-tolerance
    # bounds in sync with /ramp_state for the lifetime of the run.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        # controller_server publishes velocity commands on the relative topic
        # "cmd_vel" by default. Phase 6: rather than sending that straight to
        # the robot, it now goes to cmd_vel_nav first, which velocity_smoother
        # below watchdogs (see nav2_params.yaml's velocity_smoother comment) -
        # this remap matches nav2_bringup's own convention for the same reason.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        # Input: cmd_vel_nav (controller_server's remapped output, above).
        # Output: velocity_smoother's own topic name is the fixed
        # "cmd_vel_smoothed" (not configurable via params) - phase 8:
        # remapped to cmd_vel_gated_in, NOT straight to the robot anymore.
        # cmd_vel_watchdog_node below is now the sole publisher of the
        # real actuation topic (/model/ramp_robot/cmd_vel, the gz-sim
        # DiffDrive plugin's only listening topic - see ramp_world.launch.py's
        # GZ_CMD_VEL_TOPIC comment) - see that node's own docstring for
        # why this one-mandatory-gate pipeline shape, not a second
        # parallel publisher, is what avoids racing for the topic.
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_gated_in'),
        ],
    )
    cmd_vel_watchdog = Node(
        package='ramp_autonomy_nav', executable='cmd_vel_watchdog_node', output='screen',
    )
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[nav2_params_file],
    )
    nav2_param_switcher = Node(
        package='ramp_autonomy_nav', executable='nav2_param_switcher_node', output='screen',
    )
    # Delivery session: only meaningful when a detachable payload actually exists - launched under the same condition spawn_payload (ramp_world.launch.py) uses.
    payload_delivery = Node(
        package='ramp_autonomy_control', executable='payload_delivery_node', output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detachable_payload'), "' == 'true'"])),
    )

    return LaunchDescription([
        world_arg,
        payload_mass_arg,
        payload_cg_offset_arg,
        detachable_payload_arg,
        cargo_mode_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        headless_arg,
        robot_state_publisher,
        world,
        state_machine,
        perception,
        stability_monitor,
        descent_brake,
        motion_stall_detector,
        ekf_node,
        controller_server,
        velocity_smoother,
        cmd_vel_watchdog,
        nav2_lifecycle_manager,
        nav2_param_switcher,
        payload_delivery,
    ])
