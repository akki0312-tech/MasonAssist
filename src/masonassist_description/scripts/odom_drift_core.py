#!/usr/bin/env python3
"""ROS-free drifting odometry for MasonAssist.

Gazebo's OdometryPublisher reports the robot's true pose. Real track
odometry is not that good: tracks slip, and skid-steer turning slips a lot.
This integrates the TRUE motion, increment by increment, with the errors a
tracked robot's odometry makes:

  - distance scale error   (track slip / effective radius)   LINEAR_SCALE_ERR
  - turning scale error    (skid-steer slip while turning)   YAW_SCALE_ERR
  - heading bias per metre (uneven slip between the tracks),
    a slow random walk, seeded so runs are repeatable        YAW_BIAS_WALK

Nothing changes while the robot is still - track odometry only drifts when
the tracks move. With all errors zero the output equals the input.
"""

import math
import random

LINEAR_SCALE_ERR = 0.03          # +3 % distance
YAW_SCALE_ERR = 0.06             # +6 % turning
YAW_BIAS_WALK = 0.01             # rad/m bias random walk, per sqrt(metre)
SEED = 1


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomDrift:

    def __init__(self, linear_scale_err=LINEAR_SCALE_ERR, yaw_scale_err=YAW_SCALE_ERR,
                 yaw_bias_walk=YAW_BIAS_WALK, seed=SEED):
        self.k_lin = linear_scale_err
        self.k_yaw = yaw_scale_err
        self.walk = yaw_bias_walk
        self.rng = random.Random(seed)
        self.bias = 0.0              # rad per metre driven
        self.last_true = None        # (x, y, yaw)
        self.pose = None             # drifting (x, y, yaw), starts equal to the truth

    def update(self, x, y, yaw):
        """Feed one true pose; returns the drifting pose (x, y, yaw)."""
        if self.last_true is None:
            self.last_true = (x, y, yaw)
            self.pose = (x, y, yaw)
            return self.pose
        lx, ly, lyaw = self.last_true
        # true increment in the robot's own frame at the previous pose
        c, s = math.cos(lyaw), math.sin(lyaw)
        dx_b = c * (x - lx) + s * (y - ly)
        dy_b = -s * (x - lx) + c * (y - ly)
        dyaw = _wrap(yaw - lyaw)
        ds = math.hypot(dx_b, dy_b)
        self.last_true = (x, y, yaw)
        if ds == 0.0 and dyaw == 0.0:
            return self.pose
        if ds > 0.0 and self.walk > 0.0:
            self.bias += self.rng.gauss(0.0, self.walk * math.sqrt(ds))
        # the same increment as odometry would measure it
        dx_b *= 1.0 + self.k_lin
        dy_b *= 1.0 + self.k_lin
        dyaw = dyaw * (1.0 + self.k_yaw) + self.bias * ds
        px, py, pyaw = self.pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        self.pose = (px + c * dx_b - s * dy_b, py + s * dx_b + c * dy_b, _wrap(pyaw + dyaw))
        return self.pose

    def twist(self, v, w):
        """Body velocities as odometry would report them."""
        return v * (1.0 + self.k_lin), w * (1.0 + self.k_yaw) + self.bias * abs(v)


def compose_se2(a, b):
    """Planar pose composition a * b: b (x, y, yaw) expressed in a's frame,
    returned in a's parent frame - e.g. map->odom * odom->base = map->base."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, _wrap(ath + bth))
