#!/usr/bin/env python3
"""ROS-free global route planner over the KNOWN site layout.

Grid A* on worker_mover.OBSTACLES (the authored world geometry, the same
registry the dashboard draws), each box inflated by INFLATION_M. Returns a
short list of waypoints (line-of-sight pruned) for vfh_plus_node to follow
one at a time.

Why this exists: VFH+ is purely local. Measured live, a goal that sits
behind a convex corner (e.g. leaving the partition corridor toward the
south-west storage strip) made the robot hug the corner at exactly
r_effective, where blocked/recovery/OK ticks cycled indefinitely. Routing
through waypoints that already keep >= INFLATION_M from every known
obstacle removes that local minimum. VFH+ still does all collision
avoidance, including anything NOT in this map (workers, debris).

This is planning on an authored map, not SLAM or Nav2.
"""

import heapq
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worker_mover import OBSTACLES  # noqa: E402

RESOLUTION_M = 0.1
BOUNDS = (-8.0, 8.0)          # site perimeter (walls at +-8)
INFLATION_M = 0.7             # VFH+ r_effective (0.5975 m) + 0.1 m margin: at exactly
                              # r_effective a thin post (warning-sign pole) flickered in
                              # and out of scans and stalled the robot beside it


def _blocked(x, y, inflation=INFLATION_M):
    for cx, cy, hx, hy in OBSTACLES.values():
        dx = max(abs(x - cx) - hx, 0.0)
        dy = max(abs(y - cy) - hy, 0.0)
        if math.hypot(dx, dy) < inflation:
            return True
    return False


class SitePlanner:

    def __init__(self, resolution=RESOLUTION_M, inflation=INFLATION_M):
        self.res = resolution
        self.inflation = inflation
        lo, hi = BOUNDS
        self.n = int(round((hi - lo) / resolution)) + 1
        self.lo = lo
        self.free = [[not _blocked(self._x(i), self._x(j), inflation)
                      for j in range(self.n)] for i in range(self.n)]
        self.dynamic = None   # bool grid: learned obstacles (live_map), already inflated

    def set_dynamic(self, mask):
        """mask: n x n bool grid of learned obstacle cells (LiveMap.persistent),
        or None. Inflated here by the same INFLATION_M as the static layout."""
        if mask is None or not mask.any():
            self.dynamic = None
            return
        r = int(math.ceil(self.inflation / self.res))
        padded = np.pad(mask, r)
        out = np.zeros_like(mask)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    out |= padded[r + di:r + di + self.n, r + dj:r + dj + self.n]
        self.dynamic = out

    def _is_free(self, i, j):
        return self.free[i][j] and (self.dynamic is None or not self.dynamic[i, j])

    def route_blocked(self, start, route):
        """True if the remaining route (from the current pose through every
        waypoint) crosses a learned obstacle. Samples within INFLATION_M +
        0.1 m of the start are skipped, so passing close to something that
        is already beside the robot doesn't count as 'blocked'."""
        if self.dynamic is None or not route:
            return False
        pts = [tuple(start)] + [tuple(p) for p in route]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            steps = max(int(math.hypot(x1 - x0, y1 - y0) / (self.res / 2)), 1)
            for k in range(steps + 1):
                x, y = x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps
                if math.hypot(x - start[0], y - start[1]) < self.inflation + 0.1:
                    continue
                i, j = self._cell(x, y)
                if self.dynamic[i, j]:
                    return True
        return False

    def _x(self, i):
        return self.lo + i * self.res

    def _cell(self, x, y):
        clamp = lambda v: min(max(int(round((v - self.lo) / self.res)), 0), self.n - 1)  # noqa: E731
        return clamp(x), clamp(y)

    def _nearest_free(self, c):
        if self._is_free(c[0], c[1]):
            return c
        best, best_d = None, math.inf
        for i in range(self.n):
            for j in range(self.n):
                if self._is_free(i, j):
                    d = (i - c[0]) ** 2 + (j - c[1]) ** 2
                    if d < best_d:
                        best, best_d = (i, j), d
        return best

    def _line_free(self, a, b):
        (x0, y0), (x1, y1) = a, b
        steps = max(int(math.hypot(x1 - x0, y1 - y0) / (self.res / 2)), 1)
        for k in range(steps + 1):
            t = k / steps
            i, j = self._cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if not self._is_free(i, j):
                return False
        return True

    def plan(self, start, goal):
        """Waypoints from start to goal (goal included, start excluded), or
        None if no route exists. If start/goal sit inside the inflated
        margin (e.g. robot already close to a wall), the search begins/ends
        at the nearest free cell, and the exact goal is still appended."""
        s = self._nearest_free(self._cell(*start))
        g = self._nearest_free(self._cell(*goal))
        if s is None or g is None:
            return None
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
        h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])  # noqa: E731
        openq = [(h(s), 0.0, s)]
        came = {s: None}
        cost = {s: 0.0}
        while openq:
            _, c_cost, c = heapq.heappop(openq)
            if c == g:
                break
            if c_cost > cost[c]:
                continue
            for di, dj, w in moves:
                nb = (c[0] + di, c[1] + dj)
                if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                    continue
                if not self._is_free(nb[0], nb[1]):
                    continue
                nc = c_cost + w
                if nc < cost.get(nb, math.inf):
                    cost[nb] = nc
                    came[nb] = c
                    heapq.heappush(openq, (nc + h(nb), nc, nb))
        if g not in came:
            return None

        cells = []
        c = g
        while c is not None:
            cells.append(c)
            c = came[c]
        cells.reverse()
        pts = [(self._x(i), self._x(j)) for i, j in cells]

        # line-of-sight pruning, starting from the real start position
        route, anchor, k = [], tuple(start), 0
        while k < len(pts) - 1:
            far = k + 1
            for m in range(len(pts) - 1, k, -1):
                if self._line_free(anchor if not route else route[-1], pts[m]):
                    far = m
                    break
            route.append(pts[far])
            k = far
        if not route or math.hypot(route[-1][0] - goal[0], route[-1][1] - goal[1]) > 1e-6:
            route.append(tuple(goal))
        if len(route) >= 2 and math.hypot(route[-2][0] - goal[0], route[-2][1] - goal[1]) < 0.3:
            route.pop(-2)
        return [(round(x, 2), round(y, 2)) for x, y in route]
