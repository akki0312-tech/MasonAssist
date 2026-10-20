import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ----------------------------------------------------------------
    # Locate the installed package share directory.
    # This resolves through the workspace install/ when the workspace
    # is sourced — so package:// URIs in the URDF will resolve
    # correctly in the same environment.
    # ----------------------------------------------------------------
    pkg_share = get_package_share_directory('masonassist_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'masonassist.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'masonassist.rviz')

    # ----------------------------------------------------------------
    # Expand the Xacro at launch time using Command().
    # This is the standard safe mechanism — no inline XML injection
    # via shell arguments, which causes ROS argument parsing failures.
    # ----------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # ----------------------------------------------------------------
    # robot_state_publisher
    # Publishes /robot_description and broadcasts TF transforms
    # for all links based on joint states.
    # ----------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ----------------------------------------------------------------
    # joint_state_publisher (headless — joint_state_publisher_gui
    # is not installed in this environment).
    # Publishes zero joint states so robot_state_publisher can
    # compute and broadcast all link transforms.
    # ----------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    # ----------------------------------------------------------------
    # RViz2
    # Launched with a bundled config that pre-sets:
    #   - Fixed Frame: base_link
    #   - RobotModel display (subscribes to /robot_description)
    #   - TF display
    #   - Camera at sensible initial position
    # ----------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
