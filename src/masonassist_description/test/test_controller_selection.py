#!/usr/bin/env python3
"""Unit tests for the enable_vfh_plus / enable_obstacle_avoidance mutual
exclusion guard in launch/gazebo.launch.py.

Scope: tests ONLY check_controller_mutual_exclusion(), the OpaqueFunction
body added to reject enabling both controllers at once (both publish to
/cmd_vel). Does not launch Gazebo, does not start any ROS node, and does
not run `ros2 launch` - launch/gazebo.launch.py is loaded directly via
importlib (it can't be `import`ed normally: ROS 2 launch files are named
`*.launch.py`, which is not a valid Python module name) and the guard
function is called with a bare launch.LaunchContext(), exactly as
OpaqueFunction itself would call it, but without visiting any other
action in the file (so gazebo_sim/the controller Node actions never run).

ENVIRONMENT REQUIREMENT: this file imports the `launch` package (ROS 2
launch APIs only - no rclpy, no Gazebo), which requires:

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest test/test_controller_selection.py -v

Launch-wiring check (that generate_launch_description() actually places
check_controller_mutual_exclusion ahead of gazebo_sim/the controller
nodes, so it can raise before they start) is covered separately below
in test_validation_action_precedes_gazebo_and_controllers, by inspecting
the returned LaunchDescription's entity order - not by launching Gazebo.
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

_LAUNCH_FILE = os.path.join(os.path.dirname(__file__), "..", "launch", "gazebo.launch.py")

_spec = importlib.util.spec_from_file_location("gazebo_launch_under_test", _LAUNCH_FILE)
gazebo_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gazebo_launch)


def make_context(enable_vfh_plus, enable_obstacle_avoidance):
    context = LaunchContext()
    context.launch_configurations["enable_vfh_plus"] = enable_vfh_plus
    context.launch_configurations["enable_obstacle_avoidance"] = enable_obstacle_avoidance
    return context


def test_neither_enabled_is_accepted():
    result = gazebo_launch.check_controller_mutual_exclusion(make_context("false", "false"))
    assert result == []


def test_only_vfh_plus_enabled_is_accepted():
    result = gazebo_launch.check_controller_mutual_exclusion(make_context("true", "false"))
    assert result == []


def test_only_obstacle_avoidance_enabled_is_accepted():
    result = gazebo_launch.check_controller_mutual_exclusion(make_context("false", "true"))
    assert result == []


def test_both_enabled_is_rejected_with_useful_message():
    with pytest.raises(RuntimeError) as excinfo:
        gazebo_launch.check_controller_mutual_exclusion(make_context("true", "true"))
    message = str(excinfo.value)
    assert "enable_vfh_plus" in message
    assert "enable_obstacle_avoidance" in message
    assert "/cmd_vel" in message


def test_validation_action_precedes_gazebo_and_controllers():
    """Confirms the wiring, not just the guard function in isolation:
    generate_launch_description() must place the OpaqueFunction guard
    before gazebo_sim (IncludeLaunchDescription) and before both
    controller Node actions, so a rejection happens before any of them
    execute. Inspects the returned LaunchDescription's entity list/order
    directly - does not call ros2 launch and does not start Gazebo.
    """
    entities = gazebo_launch.generate_launch_description().entities

    def index_of(predicate):
        for i, entity in enumerate(entities):
            if predicate(entity):
                return i
        raise AssertionError("expected entity not found in LaunchDescription")

    guard_index = index_of(
        lambda e: isinstance(e, OpaqueFunction) and e._OpaqueFunction__function
        is gazebo_launch.check_controller_mutual_exclusion
    )
    gazebo_index = index_of(lambda e: isinstance(e, IncludeLaunchDescription))
    obstacle_node_index = index_of(
        lambda e: isinstance(e, Node) and e._Node__node_executable == "simple_obstacle_avoidance.py"
    )
    vfh_node_index = index_of(
        lambda e: isinstance(e, Node) and e._Node__node_executable == "vfh_plus_node.py"
    )

    assert guard_index < gazebo_index
    assert guard_index < obstacle_node_index
    assert guard_index < vfh_node_index
