import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ----------------------------------------------------------------
# enable_vfh_plus / enable_obstacle_avoidance mutual exclusion guard
#
# Both vfh_plus_node.py and simple_obstacle_avoidance.py publish to the
# same unnamespaced /cmd_vel topic, so running both at once would make
# them fight over robot control. Module-level (not nested in
# generate_launch_description) so it can be imported directly by
# test/test_controller_selection.py without executing the rest of this
# launch file. Evaluated via OpaqueFunction, since LaunchConfiguration
# values are only resolved at launch time, not when this module is
# imported or generate_launch_description() is called.
# ----------------------------------------------------------------
def check_controller_mutual_exclusion(context, *args, **kwargs):
    vfh_enabled = IfCondition(LaunchConfiguration('enable_vfh_plus')).evaluate(context)
    obstacle_enabled = IfCondition(LaunchConfiguration('enable_obstacle_avoidance')).evaluate(context)
    if vfh_enabled and obstacle_enabled:
        raise RuntimeError(
            "Invalid launch configuration: enable_vfh_plus:=true and enable_obstacle_avoidance:=true "
            "were both given, but vfh_plus_node.py and simple_obstacle_avoidance.py both publish to "
            "/cmd_vel. Enable at most one of enable_vfh_plus / enable_obstacle_avoidance (or leave both "
            "false for manual /cmd_vel control)."
        )
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('masonassist_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', 'masonassist.urdf.xacro')
    sdf_model_path = os.path.join(pkg_share, 'models', 'masonassist', 'model.sdf')

    # ----------------------------------------------------------------
    # Set GZ_SIM_RESOURCE_PATH so Gazebo Harmonic finds meshes via:
    # model://masonassist_description/meshes/...
    # package://masonassist_description/meshes/...
    # ----------------------------------------------------------------
    parent_share = os.path.dirname(pkg_share)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_resource_path:
        gz_resource_path = f'{parent_share}:{pkg_share}:{existing_resource_path}'
    else:
        gz_resource_path = f'{parent_share}:{pkg_share}'

    set_env_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # ----------------------------------------------------------------
    # Launch Arguments
    # ----------------------------------------------------------------
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position (m)')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position (m)')
    declare_z = DeclareLaunchArgument('z', default_value='0.1', description='Initial Z position (m) - slightly above ground to settle under gravity')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw orientation (rad)')
    declare_enable_obstacle_avoidance = DeclareLaunchArgument(
        'enable_obstacle_avoidance', default_value='false',
        description='Start simple_obstacle_avoidance.py, which publishes /cmd_vel. '
                     'Default false preserves normal manual /cmd_vel control with no '
                     'competing publisher.')
    declare_enable_vfh_plus = DeclareLaunchArgument(
        'enable_vfh_plus', default_value='false',
        description='Start vfh_plus_node.py, which publishes /cmd_vel. Default false '
                     'preserves normal manual /cmd_vel control with no competing '
                     'publisher. Do not enable together with enable_obstacle_avoidance - '
                     'both publish /cmd_vel and would fight each other.')
    declare_world_file = DeclareLaunchArgument(
        'world_file', default_value='masonassist.world',
        description='World file to load, resolved under worlds/. Default is the '
                     'baseline masonassist.world - unaffected unless overridden '
                     '(e.g. vfh_plus_validation.world for VFH+ behavioral testing).')

    # ----------------------------------------------------------------
    # Expand Xacro into robot_description string parameter
    # ----------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # ----------------------------------------------------------------
    # robot_state_publisher
    # ----------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ----------------------------------------------------------------
    # joint_state_publisher
    #
    # Required for robot_state_publisher to publish base_link->left_track and
    # base_link->right_track at all, since those joints are `continuous` (not
    # fixed) in the Xacro. Diagnosed directly: without a joint_state_publisher
    # of its own, this launch file had no source for /joint_states, so those
    # two TF edges were only ever appearing because of an unrelated, pre-
    # existing joint_state_publisher process left over from outside this
    # launch - and that stray process uses use_sim_time:=false, so it was
    # stamping those transforms with wall-clock time while the rest of the
    # TF tree uses sim time, breaking full-chain lookups
    # ("extrapolation into the past"). This node makes the launch
    # self-contained and sim-time-consistent.
    # ----------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------------------------------------------
    # Reject enable_vfh_plus + enable_obstacle_avoidance both true, before
    # Gazebo or any controller node starts (see check_controller_mutual_
    # exclusion above). Placed ahead of gazebo_sim/the node actions in the
    # LaunchDescription returned below, so it executes and can raise before
    # they do.
    # ----------------------------------------------------------------
    validate_controller_selection = OpaqueFunction(function=check_controller_mutual_exclusion)

    # ----------------------------------------------------------------
    # Gazebo Harmonic simulation (gz_sim.launch.py)
    # -r starts the physics simulation immediately on launch
    # ----------------------------------------------------------------
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r '),
                PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world_file')]),
            ],
        }.items(),
    )

    # ----------------------------------------------------------------
    # Spawn MasonAssist into Gazebo Harmonic via ros_gz_sim create
    #
    # Spawned from a dedicated hand-authored SDF model (models/masonassist/model.sdf),
    # NOT from -topic robot_description. Experimentally proven (see project history):
    # spawning the Xacro-derived robot_description via -topic causes libsdformat's
    # URDF importer to lump the fixed left_track/right_track joints into base_link,
    # so TrackedVehicle/TrackController can never find those links. The dedicated
    # SDF model preserves base_link/left_track/right_track as separate physics links.
    # ----------------------------------------------------------------
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_masonassist',
        output='screen',
        arguments=[
            '-name', 'masonassist',
            '-file', sdf_model_path,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ----------------------------------------------------------------
    # Clock Bridge (Gazebo -> ROS 2)
    # Bridges /clock so ROS 2 nodes with use_sim_time=True receive sim time
    # ----------------------------------------------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/world/masonassist_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/masonassist_world/clock', '/clock'),
        ],
    )

    # ----------------------------------------------------------------
    # cmd_vel / odometry Bridge (Gazebo <-> ROS 2)
    #
    # Message-type pairings verified against the actual running installation
    # (gz topic -i) rather than assumed:
    #   gz.msgs.Twist    <-> geometry_msgs/msg/Twist
    #   gz.msgs.Odometry <-> nav_msgs/msg/Odometry
    #
    # /model/masonassist/tf (gz.msgs.Pose_V) is deliberately NOT bridged to
    # ROS /tf. Verified by direct inspection: the TrackedVehicle plugin
    # publishes that topic with the per-pose "name" field unset, so the
    # bridged tf2_msgs/msg/TFMessage always has an empty child_frame_id.
    # Confirmed this is actively harmful, not just unused: it floods any
    # tf2 consumer (observed directly in RViz's own console) with continuous
    # "TF_NO_CHILD_FRAME_ID: Ignoring transform ... because child_frame_id
    # not set" errors. robot_state_publisher (above) remains the sole /tf
    # and /tf_static publisher, owning base_footprint->base_link (static)
    # and base_link->{left,right}_track (dynamic, from the Xacro's
    # continuous joints) - no duplicate/conflicting publisher for any edge.
    # ----------------------------------------------------------------
    masonassist_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='masonassist_bridge',
        output='screen',
        arguments=[
            '/model/masonassist/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # OdometryPublisher output (see model.sdf) - TrackedVehicle's own
            # /model/masonassist/odometry drifts badly under skid-steer slip.
            '/model/masonassist/odometry_sim@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/masonassist/cmd_vel', '/cmd_vel'),
            ('/model/masonassist/odometry_sim', '/odom'),
        ],
    )

    # ----------------------------------------------------------------
    # odom -> base_footprint TF broadcaster
    #
    # Gazebo's own /model/masonassist/tf cannot provide this edge (verified:
    # its Pose_V entries carry no entity name, so the bridged TFMessage
    # always has an empty child_frame_id - not bridged, see masonassist_bridge
    # above). /odom is well-formed, so this node re-broadcasts its pose as
    # the dynamic odom->base_footprint TF edge instead. It does not touch
    # /odom, does not touch left_track/right_track TF (those remain owned by
    # robot_state_publisher), and does not re-publish Gazebo's /tf.
    # ----------------------------------------------------------------
    odom_tf_broadcaster_node = Node(
        package='masonassist_description',
        executable='odom_to_tf_broadcaster.py',
        name='odom_to_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------------------------------------------
    # LiDAR Bridge (Gazebo -> ROS 2)
    #
    # Type pairing verified against the actual running installation:
    #   gz.msgs.LaserScan <-> sensor_msgs/msg/LaserScan
    # Bridged as /scan_raw, not directly as /scan: verified directly that
    # gpu_lidar always stamps its message with Gazebo's internal
    # "<model>::<link>::<sensor>" frame name (no SDF element overrides this),
    # which does not match the "laser_frame" TF frame robot_state_publisher
    # publishes. scan_frame_fix (below) corrects this the same way
    # odom_to_tf_broadcaster corrects the TF gap - not touching Gazebo's own
    # message here, only bridging it under a different name.
    # ----------------------------------------------------------------
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/scan_raw@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
    )

    scan_frame_fix_node = Node(
        package='masonassist_description',
        executable='scan_frame_fix.py',
        name='scan_frame_fix',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------------------------------------------
    # Simple obstacle avoidance (proof-of-concept closed loop, /scan -> /cmd_vel)
    #
    # Off by default (enable_obstacle_avoidance:=false) so normal manual
    # /cmd_vel control has exactly one publisher during the standard baseline
    # launch. Only started, and only then publishing /cmd_vel, when
    # explicitly enabled.
    # ----------------------------------------------------------------
    obstacle_avoidance_node = Node(
        package='masonassist_description',
        executable='simple_obstacle_avoidance.py',
        name='simple_obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_obstacle_avoidance')),
    )

    # ----------------------------------------------------------------
    # VFH+ obstacle avoidance (/scan + /odom -> /cmd_vel via VFHPlusCore)
    #
    # Off by default (enable_vfh_plus:=false), same pattern as
    # simple_obstacle_avoidance above, and mutually exclusive with it in
    # practice: both publish /cmd_vel, so only one should ever be enabled
    # for a given launch.
    # ----------------------------------------------------------------
    vfh_plus_node = Node(
        package='masonassist_description',
        executable='vfh_plus_node.py',
        name='vfh_plus_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_vfh_plus')),
    )

    return LaunchDescription([
        set_env_resource_path,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_enable_obstacle_avoidance,
        declare_enable_vfh_plus,
        declare_world_file,
        validate_controller_selection,
        gazebo_sim,
        robot_state_publisher_node,
        joint_state_publisher_node,
        clock_bridge,
        masonassist_bridge,
        odom_tf_broadcaster_node,
        lidar_bridge,
        scan_frame_fix_node,
        obstacle_avoidance_node,
        vfh_plus_node,
        spawn_robot_node,
    ])
