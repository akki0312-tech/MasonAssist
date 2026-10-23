import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression, TextSubstitution)
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
    declare_require_operator_link = DeclareLaunchArgument(
        'require_operator_link', default_value='true',
        description='safety_supervisor stops the robot if the mission server heartbeat '
                    '(/operator_heartbeat) is lost for 2 s. Set false when driving '
                    'without mission_server.py (e.g. manual /cmd_vel_nav tests).')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='amcl',
        description="'none' = map->odom is identity (pose = odometry); 'slam' = slam_toolbox "
                    "builds a new map; 'amcl' = localise against the saved map")
    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_share, 'maps', 'construction_site.yaml'),
        description='saved map for localization:=amcl')
    declare_payload = DeclareLaunchArgument(
        'payload_kg', default_value='0',
        description='mass of a loaded pallet fixed on the deck (0 = empty robot)')
    declare_payload_cog = DeclareLaunchArgument(
        'payload_cog_height', default_value='',
        description="load centre above the deck top in m ('' = resting on its pallet)")
    declare_odometry = DeclareLaunchArgument(
        'odometry', default_value='drift',
        description="'perfect' = /odom is Gazebo's true pose; 'drift' = realistic track "
                    'odometry errors (see odom_drift_core.py)')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = run the Gazebo server only (no 3D GUI window). In this VM '
                    'the software-rendered GUI alone used ~2 of 4 CPU cores and held the '
                    'simulation to ~0.57x real time; the mission dashboard shows the robot, '
                    'LiDAR and missions without it.')
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
                PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
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
    # payload_kg > 0: spawn the model with a loaded pallet fixed on the deck
    # (scripts/payload_model.py), written to a temp copy of the model.
    def spawn_with_payload(context):
        sys.path.insert(0, os.path.join(pkg_share, '..', '..', 'lib', 'masonassist_description'))
        from payload_model import model_with_payload
        mass = float(LaunchConfiguration('payload_kg').perform(context))
        cog = LaunchConfiguration('payload_cog_height').perform(context)
        path = sdf_model_path
        if mass > 0:
            fd, path = tempfile.mkstemp(prefix='masonassist_payload_', suffix='.sdf')
            with os.fdopen(fd, 'w') as f:
                f.write(model_with_payload(open(sdf_model_path).read(), mass, float(cog) if cog else None))
        return [Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_masonassist',
            output='screen',
            arguments=[
                '-name', 'masonassist',
                '-file', path,
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        )]
    spawn_robot_node = OpaqueFunction(function=spawn_with_payload)

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
    # set_pose service bridge (ROS 2 -> Gazebo), used by worker_mover.py to
    # move the construction workers through one persistent client instead of
    # spawning a `gz service` CLI process per worker per tick (that cost
    # ~0.25 s per call and held the simulation to ~0.66x real time).
    # ----------------------------------------------------------------
    world_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_service_bridge',
        output='screen',
        arguments=[
            '/world/masonassist_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
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
            # tilt sensor (model.sdf) for the safety supervisor's tilt guard
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/model/masonassist/cmd_vel', '/cmd_vel'),
            # Gazebo's true pose; odom_drift (below) turns it into /odom
            ('/model/masonassist/odometry_sim', '/ground_truth/odom'),
        ],
    )

    # ----------------------------------------------------------------
    # Odometry: /ground_truth/odom -> /odom. odometry:=drift adds the errors
    # real track odometry makes (odom_drift_core.py); odometry:=perfect
    # passes the ground truth through. Only evaluation tools read
    # /ground_truth/odom - the robot's own stack only sees /odom.
    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
    # Localisation: who provides map->odom. The robot spawns at the world
    # origin, so the map frame lines up with the site coordinates the
    # planner and dashboard use. map_pose_publisher turns map->odom + /odom
    # into /robot_pose, which navigation and the mission server read.
    # ----------------------------------------------------------------
    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_odom_identity',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'none')),
    )
    # slam_toolbox is a lifecycle node on Jazzy: its own launch file
    # configures and activates it
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(pkg_share, 'config', 'slam_mapping.yaml'),
                          'use_sim_time': 'true'}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'slam')),
    )
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map')}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'amcl')),
    )
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'amcl.yaml'), {'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'amcl')),
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization'), 'amcl')),
    )
    map_pose_node = Node(
        package='masonassist_description', executable='map_pose_publisher.py',
        name='map_pose_publisher', output='screen', parameters=[{'use_sim_time': True}],
    )

    odom_drift_node = Node(
        package='masonassist_description',
        executable='odom_drift.py',
        name='odom_drift',
        output='screen',
        parameters=[{'use_sim_time': True, 'mode': LaunchConfiguration('odometry')}],
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
        remappings=[('/cmd_vel', '/cmd_vel_nav')],   # goes through safety_supervisor
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
        parameters=[{'use_sim_time': True, 'pose_topic': '/robot_pose'}],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],   # goes through safety_supervisor
        condition=IfCondition(LaunchConfiguration('enable_vfh_plus')),
    )

    # ----------------------------------------------------------------
    # Safety supervisor: the only node that publishes the robot's /cmd_vel.
    # Navigation publishes /cmd_vel_nav; the supervisor forwards it at a
    # fixed 20 Hz only while e-stop/cancel/stale-sensor/stale-command/
    # operator-link/speed/collision checks pass, and publishes zero
    # otherwise. Measured before it existed: with the navigation node
    # frozen mid-drive, the robot kept driving blind for over 3 m.
    # require_operator_link: stop if mission_server's heartbeat is lost.
    # ----------------------------------------------------------------
    safety_supervisor_node = Node(
        package='masonassist_description',
        executable='safety_supervisor.py',
        name='safety_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'require_operator_link': LaunchConfiguration('require_operator_link'),
                     'payload_kg': ParameterValue(LaunchConfiguration('payload_kg'), value_type=float)}],
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
        declare_headless,
        declare_require_operator_link,
        declare_odometry,
        declare_payload,
        declare_payload_cog,
        declare_localization,
        declare_map,
        validate_controller_selection,
        gazebo_sim,
        robot_state_publisher_node,
        joint_state_publisher_node,
        clock_bridge,
        world_service_bridge,
        masonassist_bridge,
        odom_tf_broadcaster_node,
        lidar_bridge,
        scan_frame_fix_node,
        obstacle_avoidance_node,
        vfh_plus_node,
        safety_supervisor_node,
        odom_drift_node,
        static_map_odom,
        slam_node,
        map_server_node,
        amcl_node,
        localization_lifecycle,
        map_pose_node,
        spawn_robot_node,
    ])
