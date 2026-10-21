#!/usr/bin/env python3
"""MasonAssist safety supervisor node.

/cmd_vel_nav (from navigation) -> [checks] -> /cmd_vel (to the robot), at a
fixed 20 Hz regardless of whether navigation is still publishing. Rules live
in safety_supervisor_core.py. Status on /supervisor_status (JSON).

This is a software layer: on a physical robot the emergency stop must also
cut motor power in hardware.
"""

import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from safety_supervisor_core import SafetySupervisorCore  # noqa: E402

RATE_HZ = 20.0


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('require_operator_link', True)
        self.core = SafetySupervisorCore(self.get_parameter('require_operator_link').value)
        self.last_reason = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/supervisor_status', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._nav_cb, 10)
        self.create_subscription(String, '/mission_cmd', self._mission_cmd_cb, 10)
        self.create_subscription(Point, '/goal_pose', lambda _m: self.core.on_goal(), 10)
        self.create_subscription(Odometry, '/odom', lambda _m: self.core.on_odom(self._now()), 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(String, '/operator_heartbeat',
                                 lambda _m: self.core.on_heartbeat(self._now()), 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info('safety_supervisor started: /cmd_vel_nav -> /cmd_vel')

    @staticmethod
    def _now():
        # wall-clock monotonic: the watchdogs must keep running even if sim
        # time or a publisher stalls
        return time.monotonic()

    def _nav_cb(self, msg):
        self.core.on_nav_cmd(msg.linear.x, msg.angular.z, self._now())

    def _mission_cmd_cb(self, msg):
        self.core.on_mission_cmd(msg.data.strip().lower())

    def _scan_cb(self, msg):
        self.core.on_scan(msg.ranges, msg.angle_min, msg.angle_increment, self._now())

    def _tick(self):
        lin, ang, reason, clamped = self.core.output(self._now())
        out = Twist()
        out.linear.x = lin
        out.angular.z = ang
        self.cmd_pub.publish(out)
        self.status_pub.publish(String(data=json.dumps({
            'reason': reason, 'clamped': clamped,
            'front_clear_m': None if self.core.front_clear == float('inf') else round(self.core.front_clear, 2),
        })))
        if reason != self.last_reason:
            if reason == 'ok':
                self.get_logger().info('supervisor: ok')
            else:
                self.get_logger().warn(f'supervisor: {reason}')
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
