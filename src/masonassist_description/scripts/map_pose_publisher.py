#!/usr/bin/env python3
"""/robot_pose = the robot's pose in the map (site) frame.

Composes the latest map->odom transform (from SLAM or AMCL; an identity
static transform when no localisation runs) with /odom, and republishes it
as nav_msgs/Odometry in frame 'map' with /odom's twist. Navigation and the
mission server read /robot_pose, so they get the localised pose whenever
localisation is running and plain odometry otherwise.
"""

import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from odom_drift_core import compose_se2  # noqa: E402


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapPosePublisher(Node):

    def __init__(self):
        super().__init__('map_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, '/robot_pose', 20)
        self.create_subscription(Odometry, '/odom', self._cb, 20)

    def _cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', Time())
        except TransformException as ex:
            self.get_logger().warn(f'no map->odom transform yet: {ex}', throttle_duration_sec=5.0)
            return
        tr = t.transform
        p = msg.pose.pose
        x, y, th = compose_se2((tr.translation.x, tr.translation.y, _yaw(tr.rotation)),
                               (p.position.x, p.position.y, _yaw(p.orientation)))
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(th / 2)
        out.pose.pose.orientation.w = math.cos(th / 2)
        out.twist = msg.twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
