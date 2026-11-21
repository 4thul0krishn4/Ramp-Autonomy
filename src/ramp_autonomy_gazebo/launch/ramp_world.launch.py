"""
ramp_world.launch.py

Launches Gazebo with the chosen ramp world variant, bridges the core
topics (clock, IMU, both LiDARs - ramp-profiling /scan and 360 deg
obstacle /scan_360, see robot.urdf.xacro - odom, cmd_vel, contact/wheel
states) to ROS2, and spawns the robot from ramp_autonomy_description at
the Zone A start pose (x=1.0, facing +X toward the ramp).

Velocity commands go on GZ_CMD_VEL_TOPIC (/model/ramp_robot/cmd_vel),
not a bare /cmd_vel - see the ROBOT_NAME/GZ_CMD_VEL_TOPIC comment
below for why. Anything publishing velocity commands for this robot
(Nav2's controller_server included - see ramp_autonomy_bringup.launch.py)
needs to target that topic.

Usage:
    ros2 launch ramp_autonomy_gazebo ramp_world.launch.py world:=sharp
    ros2 launch ramp_autonomy_gazebo ramp_world.launch.py world:=curved payload_mass:=150.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Must match the '-name' argument passed to `ros_gz_sim create` below. The
# DiffDrive gz-sim plugin (see robot.urdf.xacro) has configurable odom_topic/
# tf_topic tags, but NOT a topic override for its cmd_vel subscription - that
# one is hardcoded by gz-sim to /model/<model_name>/cmd_vel, confirmed by
# spawning the robot and running `gz topic -l` (publishing straight to a bare
# /cmd_vel, as this launch file did before, silently went nowhere - the robot
# never moved because nothing was listening there).
ROBOT_NAME = 'ramp_robot'
GZ_CMD_VEL_TOPIC = f'/model/{ROBOT_NAME}/cmd_vel'


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('ramp_autonomy_gazebo')

    world_arg = DeclareLaunchArgument(
        'world', default_value='sharp',
        description="Ramp world variant: 'sharp' or 'curved'"
    )
    payload_mass_arg = DeclareLaunchArgument(
        'payload_mass', default_value='0.0',
        description='Payload mass in kg added to the robot base link (0.0 = unloaded run)'
    )
    payload_cg_offset_arg = DeclareLaunchArgument(
        'payload_cg_offset', default_value='0.0',
        description='Forward/back CG offset of the payload from base_link origin, in metres'
    )
    detachable_payload_arg = DeclareLaunchArgument(
        'detachable_payload', default_value='false',
        description="'true' = the payload is a separate model held by a gz-sim DetachableJoint (delivery session); 'false' (default) = lumped into the chassis as in every "
                    "validated run. Only takes effect when payload_mass > 0."
    )
    cargo_mode_arg = DeclareLaunchArgument(
        'cargo_mode', default_value='secured',
        description="'secured' (default) = today's exact behavior, unaffected by this arg. 'loose' = UNSECURED CARGO PHYSICS DEMO (opt-in, isolated from "
                    "every validated result - see docs/construction_site_design.md 'Unsecured cargo demo mode'): the payload is a separate free rigid body "
                    "resting on the chassis from spawn, never joint-constrained, able to slide/tip. Only takes effect when payload_mass > 0."
    )
    contact_diag_arg = DeclareLaunchArgument(
        'contact_diag', default_value='false',
        description="Diagnostic only, default false/unchanged everywhere: adds a <sensor type=contact> to the separate payload model's own collision "
                    "geometry (see payload_box.sdf.xacro) for post-hoc verification of cargo_mode:=loose's physical plausibility. Pure telemetry, no "
                    "dynamics/collision/friction change."
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='1.0', description='Robot spawn x (default: the gate, as in every validated run)')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.23', description='Robot spawn z (0.23 = on Zone A/gate ground; 6.68 on Floor 2)')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description="Run Gazebo server-only (-s), no GUI client. Needed on setups where the "
                    "gz-sim GUI crashes (e.g. a snap-packaged GTK/pthread symbol clash) - the "
                    "server and all sensor/physics/bridge topics work identically either way."
    )

    # Every existing world file follows the ramp_world_<name>.sdf naming
    # convention (ramp_world_sharp.sdf, ramp_world_curved.sdf, and their
    # TESTONLY friction variants) - world:=<name> maps to that pattern by
    # default, unchanged. construction_site.sdf (phase 8, session A) is the
    # one deliberate exception: its deliverable filename is fixed by its own
    # task spec (no "ramp_world_" prefix), so it's special-cased here rather
    # than renamed - this is a change to the launch file's naming LOGIC
    # only, not to either existing world file's own content.
    world_file = PathJoinSubstitution([
        gazebo_pkg, 'worlds',
        PythonExpression([
            "'construction_site.sdf' if '", LaunchConfiguration('world'),
            "' == 'construction_site' else 'ramp_world_' + '",
            LaunchConfiguration('world'), "' + '.sdf'",
        ])
    ])

    gz_args = PythonExpression([
        "'", world_file, " -r' + (' -s' if '", LaunchConfiguration('headless'), "' == 'true' else '')"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # NOTE: robot_description is generated by ramp_autonomy_description's
    # xacro (see urdf/robot.urdf.xacro) - payload_mass / payload_cg_offset
    # are passed straight through as xacro args so the same robot model
    # can be re-spawned loaded or unloaded without editing any files.
    # z = 0.23: base_footprint sits base_height/2 (0.125 m) below base_link,
    # and each wheel's center sits a further base_height/2 below base_link
    # (wheel_z=0 in the urdf), so wheel-bottom global z = spawn_z - base_height/2
    # regardless of wheel_radius. Zone A's driving surface (see
    # ramp_world_*.sdf zone_a_approach: pose z=0.05, box thickness 0.1) sits
    # at z=0.1, so spawn_z must be 0.1 + 0.125 = 0.225 for the wheels to just
    # touch it; 0.23 adds a hair of clearance so the robot settles under
    # gravity instead of spawning already sunk into the ground plane.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT_NAME,
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )

    # Delivery session: the separate payload model (see ramp_autonomy_description/sdf/payload_box.sdf.xacro). Spawned only for a loaded, detachable run; the robot's
    # DetachableJoint plugin (declared in robot.urdf.xacro under the same arg) attaches to it as soon as both exist.
    payload_sdf = Command([
        'xacro ', os.path.join(get_package_share_directory('ramp_autonomy_description'), 'sdf', 'payload_box.sdf.xacro'),
        ' payload_mass:=', LaunchConfiguration('payload_mass'), ' payload_cg_offset:=', LaunchConfiguration('payload_cg_offset'),
        ' contact_diag:=', LaunchConfiguration('contact_diag'),
    ])
    spawn_payload = Node(
        package='ros_gz_sim',
        executable='create',
        # `create` overrides any <pose> in the SDF with its own -x/-y/-z/-R: the payload goes where the URDF payload_chute_joint rests it (phase 8 side-chute redesign,
        # see robot.urdf.xacro's own chute comment for the full derivation - 4 iterations: secured Y moved from an initial ~0.7 m to 0.28 m because a 100 kg payload
        # that far off centreline tips the whole robot over, then tilt eased from -30 to -20 deg because the payload's own tilted corner, not the plate surface, was
        # the real obstacle-LiDAR clearance driver) = robot spawn (1.0, 0, 0.23) + chute joint x(-0.15) + cg_offset, chute payload-rest y(0.28), base_link 0.12 +
        # chute payload-rest z(0.24) = 0.36, tilted -20 deg (roll) to sit flush on the chute's own incline.
        arguments=['-string', payload_sdf, '-name', 'payload_box',
                   '-x', PythonExpression(["float('", LaunchConfiguration('spawn_x'), "') - 0.15 + float('", LaunchConfiguration('payload_cg_offset'), "')"]),
                   '-y', PythonExpression(["float('", LaunchConfiguration('spawn_y'), "') + 0.28"]),
                   '-z', PythonExpression(["float('", LaunchConfiguration('spawn_z'), "') + 0.36"]),
                   '-R', '-0.3491'],
        output='screen',
        # spawned whenever the REAL payload is a separate model: detachable_payload:=true (secured-until-released) OR cargo_mode:=loose (never secured) - see
        # robot.urdf.xacro's matching separate_payload property, which gates payload_link the same way.
        condition=IfCondition(PythonExpression([
            "('", LaunchConfiguration('detachable_payload'), "' == 'true' or '", LaunchConfiguration('cargo_mode'), "' == 'loose') and float('",
            LaunchConfiguration('payload_mass'), "') > 0.0"
        ])),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Phase 8: second, independent LiDAR (robot.urdf.xacro's
            # obstacle_lidar_link) purely for Nav2's local_costmap 360 deg
            # obstacle ring - see nav2_params.yaml's obstacle_layer, now
            # pointed at this topic instead of /scan (the ramp sensor).
            '/scan_360@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'{GZ_CMD_VEL_TOPIC}@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # EKF fusion (phase 8 session A/B follow-up): gz-sim's raw
            # odom -> base_footprint TF (from the DiffDrive plugin) is
            # deliberately NOT bridged into ROS2 anymore - ekf_node
            # (ramp_autonomy_bringup.launch.py, ekf.yaml) is now the sole
            # publisher of that transform on the ROS2 side, fusing wheel
            # odometry with the IMU instead of trusting raw wheel odom's
            # yaw, which live testing showed diverges from ground truth
            # under turning. The /odom TOPIC above is still bridged - the
            # EKF needs it as an input sensor source, it just no longer
            # owns the TF. gz-sim keeps publishing its own internal /tf
            # on the Gazebo side; it simply never reaches ROS2's tf2
            # buffer, so there is no dual-publisher conflict.
            #
            # Phase 6: per-wheel actual velocity, from gz-sim-joint-state
            # -publisher-system (robot.urdf.xacro) - see stability_monitor_node.py
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Delivery session: DetachableJoint's own topics (robot.urdf.xacro, detachable_payload:=true). Bridged unconditionally - harmless when the payload/plugin
            # don't exist (detachable_payload:=false, the default): the bridge just has nothing to relay, same as any other topic with no publisher/subscriber yet.
            '/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty',    # ROS -> gz only (payload_delivery_node.py's trigger)
            '/payload/state@std_msgs/msg/String[gz.msgs.StringMsg',  # gz -> ROS only (the joint's own attach/detach confirmation)
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        payload_mass_arg,
        payload_cg_offset_arg,
        detachable_payload_arg,
        cargo_mode_arg,
        contact_diag_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        headless_arg,
        gz_sim,
        bridge,
        spawn_robot,
        spawn_payload,
    ])
