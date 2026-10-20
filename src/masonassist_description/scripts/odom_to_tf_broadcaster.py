#!/usr/bin/env python3
"""Bridges nav_msgs/Odometry on /odom into a dynamic odom -> base_footprint TF edge.

Gazebo's own TrackedVehicle /tf output cannot be used for this (its Pose_V
entries carry no entity name, so the bridged TFMessage always has an empty
child_frame_id - verified directly, not assumed). /odom itself is well-formed,
so this node re-broadcasts its pose as TF instead of touching /odom or
Gazebo's tf topic.

Frame convention: base_footprint -> base_link is xyz="0 0 0.07683" with zero
rotation - zero X/Y offset, rotation-free. The odometry's Z is always 0 (a
planar dead-reckoning integrator - it never reports height). Given both of
those facts, the odometry pose is used directly, unmodified, as the
base_footprint pose - not adjusted for the base_link height offset, since
that offset has no horizontal or rotational component to correct for, and
adjusting Z would only move it away from base_footprint's own Z=0 convention.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
