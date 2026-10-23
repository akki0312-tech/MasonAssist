"""Minimal stack for vehicle terrain tests (tools/terrain_test.py).

Gazebo with worlds/terrain_test.world, the robot (optionally loaded:
payload_kg), and only the bridges the test harness needs: /cmd_vel in,
/ground_truth/odom (full 3-D pose) and /clock out, and the set_pose
service to place the robot at each lane. No navigation and no safety
supervisor: this measures what the vehicle itself can do on slopes.
"""
import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('masonassist_description')
    sdf_model_path = os.path.join(pkg_share, 'models', 'masonassist', 'model.sdf')
    parent_share = os.path.dirname(pkg_share)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = f'{parent_share}:{pkg_share}' + (f':{existing}' if existing else '')

    def spawn(context):
        sys.path.insert(0, os.path.join(pkg_share, '..', '..', 'lib', 'masonassist_description'))
        from payload_model import model_with_payload
        mass = float(LaunchConfiguration('payload_kg').perform(context))
        cog = LaunchConfiguration('payload_cog_height').perform(context)
        fd, path = tempfile.mkstemp(prefix='masonassist_terrain_', suffix='.sdf')
        with os.fdopen(fd, 'w') as f:
            f.write(model_with_payload(open(sdf_model_path).read(), mass, float(cog) if cog else None))
        return [Node(package='ros_gz_sim', executable='create', name='spawn_masonassist', output='screen',
                     arguments=['-name', 'masonassist', '-file', path, '-x', '-2.0', '-y', '0.0', '-z', '0.1'])]

    return LaunchDescription([
        DeclareLaunchArgument('payload_kg', default_value='0'),
        DeclareLaunchArgument('payload_cog_height', default_value=''),
        DeclareLaunchArgument('headless', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [
                TextSubstitution(text='-r '),
                PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
                os.path.join(pkg_share, 'worlds', 'terrain_test.world')]}.items()),
        OpaqueFunction(function=spawn),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='terrain_bridge', output='screen',
             arguments=['/world/masonassist_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/model/masonassist/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/model/masonassist/odometry_sim@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/world/masonassist_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
             remappings=[('/world/masonassist_world/clock', '/clock'),
                         ('/model/masonassist/cmd_vel', '/cmd_vel'),
                         ('/model/masonassist/odometry_sim', '/ground_truth/odom')]),
    ])
