#!/usr/bin/env python3
"""ROS-free human-aware speed limiting for MasonAssist.

The LiDAR cannot tell a person from a wall, but the site layout is known
(worker_mover.OBSTACLES - the same registry the route planner and dashboard
use). Returns NOT explained by that layout are treated as possible people /
dynamic objects ("unmapped"), and the robot's commanded motion is limited by
how close the nearest unmapped return is in its direction of travel:

    stop  - nearer than STOP_M: zero twist, hold until clear for RESUME_CLEAR_S
    slow  - nearer than SLOW_M: forward/reverse speed x SLOW_FACTOR
    clear - otherwise unchanged

This is unmapped-obstacle detection, not person recognition: debris left on
the floor would trigger it too, which is the conservative behaviour wanted.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from worker_mover import OBSTACLES  # noqa: E402

MAP_MATCH_M = 0.3        # return within this of a known box = part of the mapped site
STOP_M = 1.0
SLOW_M = 2.0
SLOW_FACTOR = 0.4
CORRIDOR_HALF_M = 0.6    # = VFH+ r_effective: VFH already keeps >= this lateral clearance
                         # when passing, so a wider corridor stopped the robot while it was
                         # correctly passing a stationary object (and could never clear)
ROTATE_STOP_M = 0.8      # turning in place: stop if anything unmapped is this close, any side
RESUME_CLEAR_S = 1.0

_BOXES = np.array(list(OBSTACLES.values()), dtype=float)  # cx, cy, hx, hy


def unmapped_points(robot_ranges, robot_angles, pose, range_min=0.1):
    """Robot-frame (ranges, angles) of finite returns farther than MAP_MATCH_M
    from every known obstacle box. pose = (x, y, yaw) of base_link in the
    map/odom frame (identical here: the robot spawns at the map origin)."""
    r = np.asarray(robot_ranges, dtype=float)
    a = np.asarray(robot_angles, dtype=float)
    ok = np.isfinite(r) & (r >= range_min)
    r, a = r[ok], a[ok]
    if r.size == 0:
        return r, a
    x, y, yaw = pose
    wx = x + r * np.cos(a + yaw)
    wy = y + r * np.sin(a + yaw)
    dx = np.maximum(np.abs(wx[:, None] - _BOXES[None, :, 0]) - _BOXES[None, :, 2], 0.0)
    dy = np.maximum(np.abs(wy[:, None] - _BOXES[None, :, 1]) - _BOXES[None, :, 3], 0.0)
    clearance = np.hypot(dx, dy).min(axis=1)
    keep = clearance > MAP_MATCH_M
    return r[keep], a[keep]


def classify(linear, angular, ranges, angles):
    """(zone, nearest_m) for the commanded motion against unmapped returns."""
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    if r.size == 0:
        return 'clear', None
    if abs(linear) > 1e-6:
        heading = 0.0 if linear > 0 else math.pi
        along = r * np.cos(a - heading)
        lateral = np.abs(r * np.sin(a - heading))
        in_path = (along > 0.0) & (lateral < CORRIDOR_HALF_M)
        if not np.any(in_path):
            return 'clear', None
        nearest = float(along[in_path].min())
        if nearest < STOP_M:
            return 'stop', nearest
        if nearest < SLOW_M:
            return 'slow', nearest
        return 'clear', nearest
    if abs(angular) > 1e-6:
        nearest = float(r.min())
        return ('stop' if nearest < ROTATE_STOP_M else 'clear'), nearest
    return 'clear', None


class SafetyGate:
    """Applies classify() to each motion command, with a resume delay after
    a stop so a person hovering at the threshold doesn't cause stop/go
    flicker."""

    def __init__(self):
        self.stopped_until_clear = False
        self.clear_since = None

    def filter(self, linear, angular, ranges, angles, now):
        zone, nearest = classify(linear, angular, ranges, angles)
        # while stopped, turning in place is still allowed if nothing unmapped
        # is within ROTATE_STOP_M on any side - lets VFH+ turn toward a clear
        # heading instead of waiting forever beside a stationary object
        turn = angular if classify(0.0, angular, ranges, angles)[0] == 'clear' else 0.0
        if zone == 'stop':
            self.stopped_until_clear = True
            self.clear_since = None
            return 0.0, turn, 'stop', nearest
        if self.stopped_until_clear:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since < RESUME_CLEAR_S:
                return 0.0, turn, 'stop', nearest
            self.stopped_until_clear = False
            self.clear_since = None
        if zone == 'slow':
            return linear * SLOW_FACTOR, angular, 'slow', nearest
        return linear, angular, 'clear', nearest
