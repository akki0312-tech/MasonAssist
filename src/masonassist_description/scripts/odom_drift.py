#!/usr/bin/env python3
"""MasonAssist odometry: /ground_truth/odom (Gazebo's true pose) -> /odom.

mode 'drift': /odom is what track odometry would report - errors from
odom_drift_core.py (distance and turning scale error, wandering heading
bias), repeatable via 'seed'. mode 'perfect': /odom = ground truth.
Everything downstream (TF odom->base_footprint, navigation, supervisor)
reads /odom and never sees the ground truth.
"""

import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from odom_drift_core import (  # noqa: E402
    LINEAR_SCALE_ERR, SEED, YAW_BIAS_WALK, YAW_SCALE_ERR, OdomDrift,
)


class OdomDriftNode(Node):

    def __init__(self):
        super().__init__('odom_drift')
        mode = self.declare_parameter('mode', 'drift').value
        if mode not in ('drift', 'perfect'):
            raise ValueError(f"mode must be 'drift' or 'perfect', not {mode!r}")
        drift = mode == 'drift'
        self.core = OdomDrift(
            self.declare_parameter('linear_scale_err', LINEAR_SCALE_ERR).value if drift else 0.0,
            self.declare_parameter('yaw_scale_err', YAW_SCALE_ERR).value if drift else 0.0,
            self.declare_parameter('yaw_bias_walk', YAW_BIAS_WALK).value if drift else 0.0,
            self.declare_parameter('seed', SEED).value)
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self._cb, 20)
        self.get_logger().info(
            f'odom_drift: mode={mode} (distance {self.core.k_lin:+.1%}, turning {self.core.k_yaw:+.1%}, '
            f'heading bias walk {self.core.walk} rad/m/sqrt(m))')

    def _cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y, th = self.core.update(p.x, p.y, yaw)
        v, w = self.core.twist(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(th / 2)
        out.pose.pose.orientation.w = math.cos(th / 2)
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomDriftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
