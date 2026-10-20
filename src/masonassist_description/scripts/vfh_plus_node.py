#!/usr/bin/env python3
"""ROS 2 wrapper around VFHPlusCore - the standalone, ROS-free VFH+ core.

This node owns ONLY: subscriptions, LaserScan->robot-frame conversion via
TF, watchdog/safety gating, ROS parameters, and converting the core's
selected direction into a Twist. VFHPlusCore remains the sole decision-
making algorithm - nothing here reimplements or overrides its logic.

V1 GOAL CONTRACT: there is no external goal source yet (no Nav2, no
/goal_direction). goal_direction stays fixed at 0.0 rad, meaning "straight
ahead in base_link," matching the core's own v1 assumption.

HEADING DECISION (documented per the task's explicit request to reason
about this carefully, not default silently): current_heading_error is also
kept at 0.0, even though real /odom is available here. The core's whole
coordinate convention is "0 rad = straight ahead in the robot's OWN current
frame, refreshed every call" - so "current heading relative to itself" is
trivially 0 by construction, not a quantity odometry could meaningfully
replace without redefining what the core means by that parameter. Using
/odom to construct some world-frame heading and feeding it in here would
be inventing a world-frame goal notion the core was never designed to
receive - explicitly out of scope. /odom is still subscribed to and used
for exactly one purpose: gating command publication on its staleness (see
the SAFETY section below), since a live localization/odometry feed is a
reasonable precondition for trusting the robot's own understanding of
"straight ahead" even in the current frame.

LASER FRAME HANDLING: the LiDAR mount offset (0.30 m / 0.19 m) is never
hard-coded here. Each scan's points are converted from its own
header.frame_id (laser_frame) into base_link using a live tf2 lookup, so
this node has no dependency on the exact mount geometry at all.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from vfh_plus_core import VFHParams, VFHPlusCore, VFHState  # noqa: E402


# v1 goal contract (see module docstring): fixed, not a ROS parameter -
# exposing it would invite changing it without the redesign this requires.
GOAL_DIRECTION = 0.0
CURRENT_HEADING_ERROR = 0.0


class VFHPlusNode(Node):

    def __init__(self):
        super().__init__('vfh_plus_node')

        core_defaults = VFHParams()

        # ---- Parameters -----------------------------------------------
        # Geometry fact, not a tuning knob (derived from the real chassis
        # footprint 0.725 x 0.525 m) - exposed as a parameter for
        # flexibility only, default taken from the core, not re-invented.
        self.declare_parameter('robot_radius', core_defaults.r_robot)
        # Initial/tunable, unvalidated against real data - see HANDOFF.md
        # and the VFH+ design-study history for the full discussion.
        self.declare_parameter('safety_margin', core_defaults.safety_margin)
        self.declare_parameter('d_max', core_defaults.d_max)
        # Currently UNVALIDATED against real histogram distributions.
        self.declare_parameter('threshold_high', core_defaults.threshold_high)
        self.declare_parameter('threshold_low', core_defaults.threshold_low)
        self.declare_parameter('smoothing_window', core_defaults.smoothing_window)
        self.declare_parameter('weight_goal', core_defaults.weight_goal)
        self.declare_parameter('weight_heading', core_defaults.weight_heading)
        self.declare_parameter('weight_previous', core_defaults.weight_previous)

        # Command-generation parameters (not part of VFHParams - these
        # belong to this node's Twist-conversion layer, kept separate from
        # the core's own decision logic per the task's explicit instruction).
        # forward_speed_max / max_angular_speed match the already-tested
        # propulsion baseline values used elsewhere in this project
        # (simple_obstacle_avoidance.py's forward_speed/turn_speed) - not
        # newly invented. steering_gain and scan_timeout are genuinely new
        # for this node and are NOT validated - conservative, round
        # starting points only.
        self.declare_parameter('forward_speed_max', 0.2)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('scan_timeout', 0.5)

        params = VFHParams(
            r_robot=self.get_parameter('robot_radius').value,
            safety_margin=self.get_parameter('safety_margin').value,
            d_max=self.get_parameter('d_max').value,
            range_min=core_defaults.range_min,
            range_max=core_defaults.range_max,
            sector_width_deg=core_defaults.sector_width_deg,
            smoothing_window=self.get_parameter('smoothing_window').value,
            threshold_high=self.get_parameter('threshold_high').value,
            threshold_low=self.get_parameter('threshold_low').value,
            weight_goal=self.get_parameter('weight_goal').value,
            weight_heading=self.get_parameter('weight_heading').value,
            weight_previous=self.get_parameter('weight_previous').value,
            goal_direction=GOAL_DIRECTION,
        )
        self.core = VFHPlusCore(params)

        self.forward_speed_max = self.get_parameter('forward_speed_max').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # ---- TF ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Watchdog state ---------------------------------------------
        self.last_scan_time = None
        self.last_odom_time = None

        # ---- ROS interfaces ----------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('vfh_plus_node started (r_effective=%.4f m)' % params.r_effective)

    # ---------------------------------------------------------------------

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan_time = self.get_clock().now()

        if self.last_odom_time is None or \
                (self.get_clock().now() - self.last_odom_time) > Duration(seconds=self.scan_timeout):
            self.get_logger().warn('odometry unavailable/stale - publishing zero Twist', throttle_duration_sec=2.0)
            self._publish_zero()
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f'TF lookup failed ({msg.header.frame_id} -> base_link): {ex} '
                                    '- publishing zero Twist', throttle_duration_sec=2.0)
            self._publish_zero()
            return

        robot_ranges, robot_angles = self._scan_to_robot_frame(msg, tf)

        result = self.core.process(robot_ranges, robot_angles, current_heading_error=CURRENT_HEADING_ERROR)

        if result.state != VFHState.OK:
            self.get_logger().info(f'VFH+ state={result.state.value} - publishing zero Twist',
                                    throttle_duration_sec=2.0)
            self._publish_zero()
            return

        self.cmd_vel_pub.publish(self._compute_twist(result.selected_direction))

    def watchdog_callback(self) -> None:
        """Independent of scan_callback: if scans stop arriving entirely,
        nothing would otherwise re-publish and a stale non-zero command
        would sit on /cmd_vel indefinitely. This is the fail-safe for that
        case - it only ever publishes zero, never a normal command."""
        now = self.get_clock().now()
        stale_scan = self.last_scan_time is None or (now - self.last_scan_time) > Duration(seconds=self.scan_timeout)
        stale_odom = self.last_odom_time is None or (now - self.last_odom_time) > Duration(seconds=self.scan_timeout)
        if stale_scan or stale_odom:
            self._publish_zero()

    # ---------------------------------------------------------------------

    def _scan_to_robot_frame(self, msg: LaserScan, tf) -> tuple:
        """Convert every (range, angle) in msg (its own header.frame_id,
        e.g. laser_frame) into robot-frame (base_link) polar coordinates,
        using the live TF transform - no hard-coded mount offset."""
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        bx = t.x + cos_yaw * lx - sin_yaw * ly
        by = t.y + sin_yaw * lx + cos_yaw * ly

        robot_ranges = np.hypot(bx, by)
        robot_angles = np.arctan2(by, bx)
        return robot_ranges, robot_angles

    def _compute_twist(self, selected_direction: float) -> Twist:
        """Command-generation layer, deliberately separate from the core's
        own decision logic. angular.z is a clipped proportional response to
        the selected heading error; linear.x is scaled by cos(error) so it
        falls to zero (favoring a pure pivot, not blind forward motion) as
        the required turn approaches +-90 deg, and stays zero beyond that -
        no additional arbitrary tuning beyond the exposed parameters."""
        twist = Twist()
        angular = self.steering_gain * selected_direction
        twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        twist.linear.x = self.forward_speed_max * max(0.0, math.cos(selected_direction))
        return twist

    def _publish_zero(self) -> None:
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VFHPlusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
