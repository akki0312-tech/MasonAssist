#!/usr/bin/env python3
"""ROS-free material-transport mission sequencer for MasonAssist.

A transport mission is an ordered list of legs (station, action). The
sequencer hands out exactly one goal at a time; vfh_plus_node.py does all
driving and obstacle avoidance. A leg only counts as done when the robot is
OBSERVED at the station: nav status 'reached' AND odometry within
ARRIVAL_TOLERANCE_M of that station. The odometry check matters because
vfh_plus_node keeps publishing 'reached' for the previous goal until it
processes the next one - status alone would let a stale 'reached' skip a leg.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worker_mover import OBSTACLES  # noqa: E402

ARRIVAL_TOLERANCE_M = 0.35   # vfh_plus_node GOAL_TOLERANCE_M is 0.3
WAYPOINT_REACHED_M = 0.35    # intermediate route waypoints; larger values let the robot cut corners
DWELL_S = 3.0                # simulated load/unload time (wall clock)

# No-progress detection (wall clock). The node's 'stalled' status only covers
# CONTINUOUS VFH+ blockage; seen live, the robot could also keep reporting
# 'navigating' while making no real progress (sliding along a wall toward an
# unreachable point, orbiting a goal) and would do so forever. Progress is the
# remaining route length shrinking by PROGRESS_EPS_M; without it for
# NO_PROGRESS_S the leg is replanned from the current pose, and after
# MAX_REPLANS replans without progress the mission fails cleanly.
PROGRESS_EPS_M = 0.3
NO_PROGRESS_S = 45.0
MAX_REPLANS = 2
HOLD_STATUSES = ('paused', 'estopped', 'yielding')   # robot deliberately not moving
MIN_STATION_CLEARANCE_M = 0.6  # ~ VFH+ r_effective (0.5975 m)

STATIONS = {
    'dock': (0.0, 0.0, 'dock'),
    'brick_pickup': (-6.0, -4.5, 'storage'),
    'block_pickup': (-3.0, -4.5, 'storage'),
    'building_door': (5.2, 1.25, 'delivery'),
    'scaffold': (1.0, 4.6, 'delivery'),
}

MISSIONS = {
    'bricks_to_building': {
        'material': 'bricks',
        'legs': [('brick_pickup', 'load'), ('building_door', 'unload'), ('dock', 'travel')],
    },
    'blocks_to_scaffold': {
        'material': 'blocks',
        'legs': [('block_pickup', 'load'), ('scaffold', 'unload'), ('dock', 'travel')],
    },
}


def station_clearance(x, y):
    """Distance from (x, y) to the nearest OBSTACLES box edge (0 if inside)."""
    best = math.inf
    for cx, cy, hx, hy in OBSTACLES.values():
        dx = max(abs(x - cx) - hx, 0.0)
        dy = max(abs(y - cy) - hy, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


SITE_LIMIT_M = 7.6           # perimeter walls sit at +-8 m
MIN_GOAL_CLEARANCE_M = 0.35  # ad-hoc (map-click) goals: must not be inside/against an obstacle


def validate_goal(x, y):
    """Error message for an unusable ad-hoc goal, or None if it is fine."""
    if abs(x) > SITE_LIMIT_M or abs(y) > SITE_LIMIT_M:
        return f'goal ({x:.2f}, {y:.2f}) is outside the site'
    clearance = station_clearance(x, y)
    if clearance < MIN_GOAL_CLEARANCE_M:
        return f'goal ({x:.2f}, {y:.2f}) is inside or against a known obstacle ({clearance:.2f} m clearance)'
    return None


def validate_stations():
    bad = {name: round(station_clearance(x, y), 3)
           for name, (x, y, _) in STATIONS.items()
           if station_clearance(x, y) < MIN_STATION_CLEARANCE_M}
    if bad:
        raise ValueError(f'stations too close to obstacles (clearance m): {bad}')


class TransportMission:

    def __init__(self, name, now, start_xy=None, planner=None, spec=None, points=None):
        """planner: optional site_planner.SitePlanner. With one, each leg
        is driven as a sequence of route waypoints; without, the station
        itself is the only goal (pure VFH+). spec/points let goto() build an
        ad-hoc mission whose stations are not in STATIONS."""
        spec = spec or MISSIONS[name]
        self.points = dict(points or {})
        self.name = name
        self.material = spec['material']
        self.legs = list(spec['legs'])
        self.leg_index = 0
        self.phase = 'traveling'
        self.payload = 'empty'
        self.started_at = now
        self.finished_at = None
        self.dwell_until = None
        self.abort_reason = None
        self.planner = planner
        self.route = []
        self.replans = 0
        self.best_remaining = None
        self.last_progress_t = now
        self._last_update_t = now
        self._plan_leg(start_xy, now)

    @classmethod
    def goto(cls, x, y, now, start_xy=None, planner=None):
        """One-leg travel mission to an arbitrary point (dashboard map click)."""
        spec = {'material': None, 'legs': [('goal', 'travel')]}
        return cls('goto', now, start_xy, planner, spec=spec, points={'goal': (float(x), float(y))})

    @property
    def active(self):
        return self.phase not in ('complete', 'aborted', 'failed')

    def _remaining(self, odom_xy):
        if not self.route:
            return 0.0
        total = math.hypot(odom_xy[0] - self.route[0][0], odom_xy[1] - self.route[0][1])
        for (ax, ay), (bx, by) in zip(self.route, self.route[1:]):
            total += math.hypot(bx - ax, by - ay)
        return total

    def _reset_progress(self, now):
        self.best_remaining = None
        self.last_progress_t = now

    def _check_progress(self, now, odom_xy, nav_status):
        """Returns a replanned goal, or None. May fail the mission."""
        if nav_status in HOLD_STATUSES:
            return None
        remaining = self._remaining(odom_xy)
        if self.best_remaining is None or remaining < self.best_remaining - PROGRESS_EPS_M:
            self.best_remaining = remaining
            self.last_progress_t = now
            return None
        if now - self.last_progress_t <= NO_PROGRESS_S:
            return None
        station = self.legs[self.leg_index][0]
        if self.replans >= MAX_REPLANS:
            self.phase = 'failed'
            self.abort_reason = f'no progress toward {station} after {self.replans} replans'
            self.finished_at = now
            self.route = []
            return None
        self.replans += 1
        self._plan_leg(odom_xy, now)
        return self.current_goal()

    def _xy(self, station):
        if station in self.points:
            return self.points[station]
        x, y, _ = STATIONS[station]
        return (x, y)

    def _plan_leg(self, from_xy, now):
        self._reset_progress(now)
        sx, sy = self._xy(self.legs[self.leg_index][0])
        if self.planner is None or from_xy is None:
            self.route = [(sx, sy)]
            return
        route = self.planner.plan(from_xy, (sx, sy))
        if route is None:
            self.route = []
            self.abort(f'no route to {self.legs[self.leg_index][0]}', now)
        else:
            self.route = route

    def current_goal(self):
        if not self.active or not self.route:
            return None
        return self.route[0]

    def update(self, now, odom_xy, nav_status):
        """Advance the mission. Returns a new (x, y) goal to publish, or None."""
        if not self.active:
            return None

        station, action = self.legs[self.leg_index]
        dt = now - self._last_update_t
        self._last_update_t = now
        if nav_status in HOLD_STATUSES or self.phase in ('loading', 'unloading'):
            self.last_progress_t += dt   # clock paused: not trying to move

        if self.phase in ('loading', 'unloading'):
            if now < self.dwell_until:
                return None
            self.payload = self.material if action == 'load' else 'empty'
            return self._advance(now, odom_xy)

        if odom_xy is None:
            return None

        replanned = self._check_progress(now, odom_xy, nav_status)
        if replanned is not None or not self.active:
            return replanned

        # intermediate waypoints: pure distance check, no 'reached' needed
        advanced = False
        while len(self.route) > 1 and \
                math.hypot(odom_xy[0] - self.route[0][0], odom_xy[1] - self.route[0][1]) < WAYPOINT_REACHED_M:
            self.route.pop(0)
            advanced = True
        if advanced:
            return self.route[0]
        if len(self.route) > 1:
            return None

        if nav_status != 'reached':
            return None
        sx, sy = self._xy(station)
        if math.hypot(odom_xy[0] - sx, odom_xy[1] - sy) > ARRIVAL_TOLERANCE_M:
            return None

        if action == 'travel':
            return self._advance(now, odom_xy)
        self.phase = 'loading' if action == 'load' else 'unloading'
        self.dwell_until = now + DWELL_S
        return None

    def _advance(self, now, odom_xy):
        self.leg_index += 1
        self.dwell_until = None
        if self.leg_index >= len(self.legs):
            self.phase = 'complete'
            self.finished_at = now
            self.route = []
            return None
        self.phase = 'traveling'
        self._plan_leg(odom_xy, now)
        return self.current_goal()

    def abort(self, reason, now):
        if self.active:
            self.phase = 'aborted'
            self.abort_reason = reason
            self.finished_at = now

    def to_dict(self, now):
        end = self.finished_at if self.finished_at is not None else now
        return {
            'name': self.name,
            'phase': self.phase,
            'payload': self.payload,
            'leg_index': self.leg_index,
            'legs': [{'station': s, 'action': a, 'x': self._xy(s)[0], 'y': self._xy(s)[1]}
                     for s, a in self.legs],
            'elapsed_s': round(end - self.started_at, 1),
            'abort_reason': self.abort_reason,
            'route': [{'x': x, 'y': y} for x, y in self.route],
            'replans': self.replans,
            'no_progress_s': round(max(0.0, now - self.last_progress_t), 1) if self.active else None,
        }


validate_stations()
