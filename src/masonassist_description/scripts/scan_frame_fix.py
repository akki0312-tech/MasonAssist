#!/usr/bin/env python3
"""Republishes the bridged LaserScan with a corrected header.frame_id.

Gazebo's gpu_lidar sensor always sets its message's frame to the internal,
auto-generated "<model>::<link>::<sensor>" name (confirmed by direct
inspection - there is no SDF element to override this). ros_gz_bridge's
gz-to-ros LaserScan conversion carries that name through verbatim (just
replacing "::" with "/"), so the bridged topic's frame_id never matches the
clean "laser_frame" name that robot_state_publisher publishes in TF. This
node subscribes to the raw bridged topic and republishes the same scan data
with frame_id corrected to "laser_frame", so consumers (RViz, etc.) can
resolve it against the real TF tree. It does not modify /odom or /tf, and
does not touch anything else about the scan data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameFix(Node):

    def __init__(self):
        super().__init__('scan_frame_fix')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan) -> None:
        msg.header.frame_id = 'laser_frame'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
