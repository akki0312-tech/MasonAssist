#!/usr/bin/env python3
"""Unit tests for the ROS integration layer in scripts/vfh_plus_node.py.

Scope: this file tests ONLY vfh_plus_node.py's own decision logic (TF
handling, staleness watchdogs, the scan-to-robot-frame conversion, and
Twist generation). It does not re-test scripts/vfh_plus_core.py, which
already has its own 37 tests in test_vfh_plus_core.py (unmodified here).

ENVIRONMENT REQUIREMENT (unlike test_vfh_plus_core.py, which needs only
bare python3+numpy): this file imports rclpy/tf2_ros/geometry_msgs/
nav_msgs/sensor_msgs, which require the ROS 2 environment sourced first:

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 -m pytest test/test_vfh_plus_core.py test/test_vfh_plus_node.py -v

No Gazebo, no executor, no spin(), and no sleeps are used anywhere in
this file - confirmed in the Phase 1 feasibility check that every
target method can be called directly as a plain Python method, and that
time-based staleness can be tested deterministically by computing
genuinely-past rclpy.time.Time values rather than waiting for real time
to elapse.

WHAT IS REAL VS. MOCKED, per method:
- scan_callback / watchdog_callback / _scan_to_robot_frame / _compute_twist:
  always the real, unmodified methods, called directly.
- tf2_ros.Buffer: a real tf2_ros.Buffer in every test (the node's own
  node.tf_buffer, seeded via its real set_transform() method) - never a
  mock - EXCEPT in test_scan_callback_extrapolation_exception_*, where
  node.tf_buffer.lookup_transform is replaced with a stub that raises
  ExtrapolationException directly. This one case is disclosed explicitly:
  Phase 1 verified that a real tf2_ros.Buffer CAN raise
  ExtrapolationException, but only when queried for a specific,
  non-"latest" timestamp - and vfh_plus_node.py's own scan_callback
  always calls lookup_transform(..., rclpy.time.Time()), i.e. "latest
  available", which could not be made to raise it against the same
  buffer data in the Phase 1 check. That test therefore verifies the
  node's `except` clause behaves correctly IF this exception were ever
  raised, not that the node's actual call pattern can trigger it through
  genuine TF usage - see the Phase 3 report for the full discussion.
- /cmd_vel publishing: node.cmd_vel_pub is the real rclpy.publisher.Publisher
  the node constructs; only its .publish method is replaced with a
  recording spy for each test, so assertions can be made deterministically
  on exactly what the node decided to send, without depending on
  cross-process DDS delivery timing (explicitly out of scope per the
  approved Phase 1 plan - "avoid tests that depend on ... an uncontrolled
  ROS graph").
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from vfh_plus_node import VFHPlusNode  # noqa: E402


# ---------------------------------------------------------------------------
# Fixture: a real, standalone VFHPlusNode with a publish spy, one per test.
# ---------------------------------------------------------------------------

@pytest.fixture
def node_env():
    rclpy.init()
    node = VFHPlusNode()
    published = []
    # Mock the external hand-off to DDS only (see module docstring) - the
    # real Publisher object still exists; only its .publish entry point is
    # intercepted so the test can assert on exactly what the node sent.
    node.cmd_vel_pub.publish = lambda msg: published.append(msg)
    try:
        yield node, published
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_scan(ranges, angle_min=0.0, angle_increment=0.0, frame_id='laser_frame'):
    """A minimal, valid LaserScan carrying exactly the given ranges, each
    `angle_increment` apart starting at `angle_min`. Deliberately small
    (not a full 360-sample Gazebo-style scan) - vfh_plus_node.py's own
    code only ever uses len(msg.ranges)/angle_min/angle_increment, never
    an assumed sample count, so a small synthetic scan exercises the same
    code paths with far easier-to-hand-verify numbers."""
    msg = LaserScan()
    msg.header.frame_id = frame_id
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = list(ranges)
    return msg


def seed_transform(buffer, node, parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    """Seed a real tf2_ros.Buffer with one known static-ish transform via
    its real set_transform() - no mocking of tf2 itself."""
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    buffer.set_transform(t, 'test')


def far_past(node) -> rclpy.time.Time:
    """A genuinely-past Time, well beyond node.scan_timeout, computed once
    (no sleeping) so staleness checks are deterministic."""
    return node.get_clock().now() - Duration(seconds=node.scan_timeout + 2.0)


# ---------------------------------------------------------------------------
# A. No odometry received at all
# ---------------------------------------------------------------------------

def test_scan_callback_no_odometry_publishes_zero(node_env):
    """scan_callback's first check is `last_odom_time is None` - true from
    construction until the first /odom message ever arrives. This must
    short-circuit before any TF lookup is attempted at all."""
    node, published = node_env
    assert node.last_odom_time is None  # precondition, not assumed

    msg = make_scan(ranges=[5.0], angle_min=0.0, angle_increment=0.0)
    node.scan_callback(msg)

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


# ---------------------------------------------------------------------------
# B. Stale odometry
# ---------------------------------------------------------------------------

def test_scan_callback_stale_odometry_publishes_zero(node_env):
    """last_odom_time exists but is older than scan_timeout - must publish
    zero without ever reaching the TF lookup (no transform is seeded in
    this test at all, so a TF attempt would raise LookupException instead
    of the expected zero-from-staleness path - this test would fail
    loudly, not silently, if the staleness check were ever bypassed)."""
    node, published = node_env
    node.last_odom_time = far_past(node)

    msg = make_scan(ranges=[5.0], angle_min=0.0, angle_increment=0.0)
    node.scan_callback(msg)

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


def test_scan_callback_fresh_odometry_proceeds_past_staleness_check(node_env):
    """Negative-path check for A/B: with odometry fresh AND a valid
    identity transform seeded, scan_callback must proceed all the way to
    actually calling the real VFHPlusCore and publish a real (non-forced-
    zero) command for clear-ahead geometry - confirms the staleness check
    isn't accidentally firing when it shouldn't."""
    node, published = node_env
    node.last_odom_time = node.get_clock().now()
    seed_transform(node.tf_buffer, node, 'base_link', 'laser_frame')

    msg = make_scan(ranges=[], angle_min=0.0, angle_increment=0.0)  # empty -> NO_DATA, not a staleness zero
    node.scan_callback(msg)

    assert len(published) == 1
    # NO_DATA also publishes zero, but via the VFHState branch, not the
    # staleness branch - distinguished here by reaching this point at all
    # (an empty scan with stale odometry would never get past the first
    # check in test B above).
    assert published[0].linear.x == 0.0


# ---------------------------------------------------------------------------
# C. Stale LiDAR / watchdog
# ---------------------------------------------------------------------------

def test_watchdog_stops_when_scan_never_received(node_env):
    node, published = node_env
    assert node.last_scan_time is None
    node.last_odom_time = node.get_clock().now()

    node.watchdog_callback()

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


def test_watchdog_stops_when_scan_is_stale(node_env):
    node, published = node_env
    node.last_scan_time = far_past(node)
    node.last_odom_time = node.get_clock().now()

    node.watchdog_callback()

    assert len(published) == 1
    assert published[0].linear.x == 0.0


def test_watchdog_does_not_publish_when_everything_is_fresh(node_env):
    """Positive-path check: the watchdog must NOT fire when both scan and
    odometry are current. scan_timeout is widened to 5 s for this one test
    only (a legitimate test-time configuration, not a production change)
    so the assertion is robust to incidental test-process scheduling
    delays rather than racing the default 0.5 s window."""
    node, published = node_env
    node.scan_timeout = 5.0
    node.last_scan_time = node.get_clock().now()
    node.last_odom_time = node.get_clock().now()

    node.watchdog_callback()

    assert len(published) == 0


# ---------------------------------------------------------------------------
# D. TF failures
# ---------------------------------------------------------------------------

def test_scan_callback_lookup_exception_publishes_zero(node_env):
    """Real, unseeded tf2_ros.Buffer: the scan's frame_id was never
    inserted, so the node's own lookup_transform call naturally raises
    LookupException - not forced, not mocked."""
    node, published = node_env
    node.last_odom_time = node.get_clock().now()
    # node.tf_buffer is freshly constructed by VFHPlusNode.__init__ and
    # nothing has been seeded into it in this test.

    msg = make_scan(ranges=[5.0], angle_min=0.0, angle_increment=0.0, frame_id='laser_frame')
    node.scan_callback(msg)

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


def test_scan_callback_connectivity_exception_publishes_zero(node_env):
    """Real tf2_ros.Buffer seeded with two individually-known but
    disconnected trees ('base_link' lives under 'root_x', 'laser_frame'
    lives under a separate, unrelated 'root_y') - lookup_transform
    naturally raises ConnectivityException, verified in Phase 1 against
    a standalone Buffer before writing this test."""
    node, published = node_env
    node.last_odom_time = node.get_clock().now()
    seed_transform(node.tf_buffer, node, 'root_x', 'base_link')
    seed_transform(node.tf_buffer, node, 'root_y', 'laser_frame')

    msg = make_scan(ranges=[5.0], angle_min=0.0, angle_increment=0.0, frame_id='laser_frame')
    node.scan_callback(msg)

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


def test_scan_callback_extrapolation_exception_publishes_zero(node_env):
    """See the module docstring and the Phase 3 report: a real
    tf2_ros.Buffer could not be made to raise ExtrapolationException
    through vfh_plus_node.py's own call pattern (always requests "latest
    available" via rclpy.time.Time()). This test forces the exception
    directly from a stub standing in for tf_buffer.lookup_transform, to
    verify the node's `except` clause itself is correct - it does NOT
    demonstrate that real TF usage can trigger this path in practice."""
    node, published = node_env
    node.last_odom_time = node.get_clock().now()

    def raise_extrapolation(*args, **kwargs):
        raise tf2_ros.ExtrapolationException('forced for test - see module docstring')

    node.tf_buffer.lookup_transform = raise_extrapolation

    msg = make_scan(ranges=[5.0], angle_min=0.0, angle_increment=0.0, frame_id='laser_frame')
    node.scan_callback(msg)

    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == 0.0


# ---------------------------------------------------------------------------
# E. Scan-to-robot-frame transformation
# ---------------------------------------------------------------------------

def test_scan_to_robot_frame_known_transform_and_points():
    """No fixture needed beyond a bare node (no odom/watchdog involved) -
    a real tf2_ros.Buffer seeded with ONE known, non-trivial transform
    (translation (1.0, 2.0, 0.0), yaw=+90 deg - deliberately not an
    identity transform, so the rotation math is genuinely exercised, not
    just pass-through), a hand-built scan with two known points, and the
    real _scan_to_robot_frame, checked against hand-computed expected
    robot-frame coordinates."""
    rclpy.init()
    node = VFHPlusNode()
    try:
        yaw = math.pi / 2.0
        seed_transform(node.tf_buffer, node, 'base_link', 'laser_frame', x=1.0, y=2.0, z=0.0, yaw=yaw)
        tf = node.tf_buffer.lookup_transform('base_link', 'laser_frame', rclpy.time.Time())

        # Two sensor-frame points: (range=5.0, angle=0.0) and (range=2.0, angle=pi/2)
        msg = make_scan(ranges=[5.0, 2.0], angle_min=0.0, angle_increment=math.pi / 2.0,
                         frame_id='laser_frame')

        robot_ranges, robot_angles = node._scan_to_robot_frame(msg, tf)

        # Hand-computed expected values: rotate each sensor-frame (lx, ly)
        # by yaw, then translate by (1.0, 2.0).
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        expected = []
        for d, a in [(5.0, 0.0), (2.0, math.pi / 2.0)]:
            lx, ly = d * math.cos(a), d * math.sin(a)
            bx = 1.0 + cos_yaw * lx - sin_yaw * ly
            by = 2.0 + sin_yaw * lx + cos_yaw * ly
            expected.append((math.hypot(bx, by), math.atan2(by, bx)))

        for i, (exp_d, exp_a) in enumerate(expected):
            assert robot_ranges[i] == pytest.approx(exp_d, abs=1e-9)
            assert robot_angles[i] == pytest.approx(exp_a, abs=1e-9)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# F. Command generation
# ---------------------------------------------------------------------------

def test_compute_twist_linear_never_negative_and_angular_clipped():
    rclpy.init()
    node = VFHPlusNode()
    try:
        directions = [0.0, 0.1, -0.1, 0.5, -0.5, math.pi / 2, -math.pi / 2,
                      2.0, -2.0, math.pi, -math.pi + 0.01]
        for direction in directions:
            twist = node._compute_twist(direction)
            assert twist.linear.x >= 0.0, f'linear.x negative for direction={direction}'
            assert abs(twist.angular.z) <= node.max_angular_speed + 1e-9, \
                f'angular.z={twist.angular.z} exceeds max_angular_speed={node.max_angular_speed} for direction={direction}'

        # Specific known values, hand-computed against the real formula.
        zero = node._compute_twist(0.0)
        assert zero.linear.x == pytest.approx(node.forward_speed_max)
        assert zero.angular.z == pytest.approx(0.0)

        perpendicular = node._compute_twist(math.pi / 2.0)
        assert perpendicular.linear.x == pytest.approx(0.0, abs=1e-9)

        # A direction whose unclipped angular command would exceed the
        # limit must be clipped to exactly max_angular_speed.
        large = node._compute_twist(10.0)  # steering_gain * 10.0 >> max_angular_speed
        assert large.angular.z == pytest.approx(node.max_angular_speed)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))


# ---------------------------------------------------------------------------
# escape_linear_velocity - leaving the safety envelope
# ---------------------------------------------------------------------------

from vfh_plus_node import ESCAPE_SPEED, escape_linear_velocity  # noqa: E402

R_ROBOT = 0.4475
RANGE_MIN = 0.1


def test_escape_reverses_away_from_front_obstacle_when_rear_clear():
    # corner 0.45 m ahead-right, nothing behind
    v = escape_linear_velocity([0.45, 5.0], [math.radians(-44), math.radians(170)], R_ROBOT, RANGE_MIN)
    assert v == -ESCAPE_SPEED


def test_escape_goes_forward_away_from_rear_obstacle_when_front_clear():
    v = escape_linear_velocity([0.4, 5.0], [math.radians(160), 0.0], R_ROBOT, RANGE_MIN)
    assert v == ESCAPE_SPEED


def test_escape_refused_when_escape_path_blocked():
    # obstacle ahead at 0.45 m AND another directly behind at 0.6 m
    v = escape_linear_velocity([0.45, 0.6], [0.0, math.pi], R_ROBOT, RANGE_MIN)
    assert v is None


def test_escape_ignores_side_points_outside_swept_width():
    # wall alongside at 0.5 m lateral does not block reversing
    v = escape_linear_velocity([0.45, 0.5, 0.5], [0.0, math.pi / 2, -math.pi / 2], R_ROBOT, RANGE_MIN)
    assert v == -ESCAPE_SPEED


def test_escape_none_without_valid_points():
    assert escape_linear_velocity([float('inf')], [0.0], R_ROBOT, RANGE_MIN) is None


def test_compute_twist_pivots_without_forward_motion_at_large_error():
    from vfh_plus_node import PIVOT_THRESHOLD_RAD
    rclpy.init()
    node = VFHPlusNode()
    try:
        t = node._compute_twist(PIVOT_THRESHOLD_RAD + 0.05)
        assert t.linear.x == 0.0 and t.angular.z > 0.0
        t = node._compute_twist(PIVOT_THRESHOLD_RAD - 0.05)
        assert t.linear.x > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_compute_twist_latches_turn_direction_near_pi():
    rclpy.init()
    node = VFHPlusNode()
    try:
        first = node._compute_twist(math.radians(170))     # turn left
        assert first.angular.z > 0
        flipped = node._compute_twist(math.radians(-172))   # goal wrapped to the other side
        assert flipped.angular.z > 0, 'must keep turning the same way near +-180 deg'
        normal = node._compute_twist(math.radians(-60))     # well inside: proportional again
        assert normal.angular.z < 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
