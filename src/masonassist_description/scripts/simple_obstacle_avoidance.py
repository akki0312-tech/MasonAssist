#!/usr/bin/env python3
"""Minimal LiDAR-reactive obstacle avoidance - proof of concept only.

Closes the loop: /scan -> filter -> front/left/right sector distances ->
simple threshold decision -> /cmd_vel. No path planning, no map, no memory
between scans - each incoming LaserScan is judged independently.

Decision logic (see module docstring in the report for the full explanation):
  - Ignore inf, nan, and any range outside [range_min, range_max].
  - FRONT sector (+/- front_half_angle around dead ahead): if the closest
    valid point there is nearer than stopping_distance, the path is BLOCKED.
  - When BLOCKED: compare average valid range in the LEFT sector
    (0, +side_scan_angle] against the RIGHT sector [-side_scan_angle, 0).
    Turn toward whichever side is more open. A sector with no valid
    readings at all is treated as fully open (range_max), not as blocked
    and not as an unresolvable infinity - this keeps the comparison always
    well-defined and deterministic, including the tie case (defaults to
    turning left).
  - When CLEAR: drive straight forward. No turning while moving forward in
    this first version - deliberately conservative, matching the example
    behavior requested.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('simple_obstacle_avoidance')

        self.declare_parameter('stopping_distance', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.3)
        self.declare_parameter('front_half_angle', 0.26)
        self.declare_parameter('side_scan_angle', 1.0)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def _sector_ranges(self, msg: LaserScan, angle_lo: float, angle_hi: float):
        """Valid (finite, in-range) readings whose angle falls in [angle_lo, angle_hi]."""
        valid = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if angle_lo <= angle <= angle_hi:
                valid.append(r)
        return valid

    def scan_callback(self, msg: LaserScan) -> None:
        stopping_distance = self.get_parameter('stopping_distance').value
        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value
        front_half_angle = self.get_parameter('front_half_angle').value
        side_scan_angle = self.get_parameter('side_scan_angle').value

        front = self._sector_ranges(msg, -front_half_angle, front_half_angle)
        min_front = min(front) if front else msg.range_max

        cmd = Twist()

        if min_front >= stopping_distance:
            # Path ahead is clear - drive straight.
            cmd.linear.x = forward_speed
            cmd.angular.z = 0.0
        else:
            # Blocked - stop forward motion and turn toward the more open side.
            left = self._sector_ranges(msg, front_half_angle, side_scan_angle)
            right = self._sector_ranges(msg, -side_scan_angle, -front_half_angle)

            left_score = (sum(left) / len(left)) if left else msg.range_max
            right_score = (sum(right) / len(right)) if right else msg.range_max

            cmd.linear.x = 0.0
            # Tie (including both sectors empty) defaults to turning left,
            # a fixed, deterministic choice rather than an arbitrary one.
            cmd.angular.z = turn_speed if left_score >= right_score else -turn_speed

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
