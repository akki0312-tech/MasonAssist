#!/usr/bin/env python3
"""ROS-free live obstacle map for MasonAssist.

Learns obstacles that the robot actually sees with its LiDAR but that are NOT
in the authored site layout (worker_mover.OBSTACLES) - debris, a new pallet -
so the A* planner can route around them. Occupancy grid with log-odds: each
beam lowers the cells it passes through ("free") and raises the cell it ends
in ("hit"), so cells clear again once whatever was there has gone.

Only PERSISTENT cells (occupied continuously for PERSIST_S) are handed to the
planner: workers walk past at ~1.8 m/s and pause ~2 s at lane ends; they are
handled by VFH+ and the safety gate, not by re-routing around where a person
happened to be standing.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from site_planner import BOUNDS, RESOLUTION_M  # noqa: E402
from worker_mover import OBSTACLES  # noqa: E402

LIDAR_X_OFFSET = 0.30     # laser_frame is 0.30 m ahead of base_link (live TF)
SELF_RANGE_M = 0.25       # closer returns are the robot's own track housings
MAX_RANGE_M = 6.0         # clear free space only this far along each beam
MAP_MATCH_M = 0.3         # within this of a known box = mapped site (safety_zones rule)
ROBOT_CLEAR_M = 0.5       # never mark cells this close to the robot
HIT, MISS = 0.9, -0.4
LO_MIN, LO_MAX = -2.0, 3.5
OCCUPIED = 1.5            # ~2 consecutive hits
PERSIST_S = 4.0


def _known_layout_mask(n, lo, res):
    xs = lo + np.arange(n) * res
    gx, gy = np.meshgrid(xs, xs, indexing='ij')
    mask = np.zeros((n, n), dtype=bool)
    for cx, cy, hx, hy in OBSTACLES.values():
        dx = np.maximum(np.abs(gx - cx) - hx, 0.0)
        dy = np.maximum(np.abs(gy - cy) - hy, 0.0)
        mask |= np.hypot(dx, dy) <= MAP_MATCH_M
    return mask


class LiveMap:

    def __init__(self, resolution=RESOLUTION_M):
        self.res = resolution
        self.lo = BOUNDS[0]
        self.n = int(round((BOUNDS[1] - BOUNDS[0]) / resolution)) + 1
        self.known = _known_layout_mask(self.n, self.lo, resolution)
        self.clear()

    def clear(self):
        self.logodds = np.zeros((self.n, self.n))
        self.occupied_since = np.full((self.n, self.n), np.nan)

    def _idx(self, v):
        return np.clip(np.round((v - self.lo) / self.res).astype(int), 0, self.n - 1)

    def update(self, pose, ranges, angle_min, angle_increment, now):
        """pose = (x, y, yaw) of base_link; ranges = RAW laser ranges as
        floats: +inf = nothing within range (beam passed through free space),
        -inf / nan = no usable information (e.g. below the sensor minimum in
        the track self-occlusion bands) - those beams are skipped entirely."""
        x, y, yaw = pose
        r = np.asarray(ranges, dtype=float)
        ang = yaw + angle_min + np.arange(r.size) * angle_increment
        ox = x + LIDAR_X_OFFSET * np.cos(yaw)
        oy = y + LIDAR_X_OFFSET * np.sin(yaw)

        finite = np.isfinite(r)
        self_hit = finite & (r < SELF_RANGE_M)
        hit = finite & ~self_hit & (r <= MAX_RANGE_M)
        far = (finite & (r > MAX_RANGE_M)) | np.isposinf(r)
        free_len = np.where(hit, r - self.res, np.where(far, MAX_RANGE_M, 0.0))
        free_len = np.clip(free_len, 0.0, MAX_RANGE_M)

        # free space: sample every beam at grid resolution up to its free length
        steps = np.arange(0.0, MAX_RANGE_M, self.res)
        along = steps[None, :]
        keep = along < free_len[:, None]
        fx = (ox + np.cos(ang)[:, None] * along)[keep]
        fy = (oy + np.sin(ang)[:, None] * along)[keep]
        delta = np.zeros((self.n, self.n))
        np.add.at(delta, (self._idx(fx), self._idx(fy)), MISS)
        free_touched = delta < 0
        delta = np.where(free_touched, MISS, 0.0)   # at most one MISS per cell per scan

        hx = ox + np.cos(ang[hit]) * r[hit]
        hy = oy + np.sin(ang[hit]) * r[hit]
        hits = np.zeros((self.n, self.n), dtype=bool)
        hits[self._idx(hx), self._idx(hy)] = True
        delta[hits] = HIT

        # never learn the mapped site or the robot's own surroundings
        xs = self.lo + np.arange(self.n) * self.res
        gx, gy = np.meshgrid(xs, xs, indexing='ij')
        near_robot = np.hypot(gx - x, gy - y) < ROBOT_CLEAR_M
        delta[self.known | near_robot] = 0.0

        self.logodds = np.clip(self.logodds + delta, LO_MIN, LO_MAX)
        occ = self.logodds > OCCUPIED
        newly = occ & np.isnan(self.occupied_since)
        self.occupied_since[newly] = now
        self.occupied_since[~occ] = np.nan

    def persistent(self, now):
        since = self.occupied_since
        return ~np.isnan(since) & (now - np.nan_to_num(since, nan=now) >= PERSIST_S)

    def cells(self, now):
        ii, jj = np.nonzero(self.persistent(now))
        return [[round(self.lo + i * self.res, 2), round(self.lo + j * self.res, 2)]
                for i, j in zip(ii.tolist(), jj.tolist())]
