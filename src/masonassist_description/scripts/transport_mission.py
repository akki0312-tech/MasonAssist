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
from site_model import BOUNDS, MISSIONS, SITE, STATIONS  # noqa: E402

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

# Live-map rerouting: if the remaining route crosses an obstacle learned from
# LiDAR (live_map.py via SitePlanner.set_dynamic), replan around it at once
# instead of waiting for the no-progress timer. Separate budget from replans.
ROUTE_CHECK_S = 1.0
REROUTE_MIN_GAP_S = 5.0
MIN_STATION_CLEARANCE_M = 0.6  # ~ VFH+ r_effective (0.5975 m)

# STATIONS and MISSIONS come from config/site.yaml (site_model).



def station_clearance(x, y):
    """Distance from (x, y) to the nearest known obstacle (0 if inside)."""
    return float(SITE.clearance(x, y))


SITE_LIMIT_M = BOUNDS[1] - 0.4   # goals must stay inside the perimeter walls
MIN_GOAL_CLEARANCE_M = 0.35  # ad-hoc (map-click) goals: must not be inside/against an obstacle


def validate_goal(x, y):
    """Error message for an unusable ad-hoc goal, or None if it is fine."""
    if abs(x) > SITE_LIMIT_M or abs(y) > SITE_LIMIT_M:
        return f'goal ({x:.2f}, {y:.2f}) is outside the site'
    clearance = station_clearance(x, y)
    if clearance < MIN_GOAL_CLEARANCE_M:
        return f'goal ({x:.2f}, {y:.2f}) is inside or against a known obstacle ({clearance:.2f} m clearance)'
    return None


MIN_JOB_SEPARATION_M = 0.5  # custom job: pick-up and drop-off must differ


def custom_job_spec(pickup, dropoff, planner=None):
    """Operator-defined job: load at `pickup`, unload at `dropoff`, return to
    the dock. Each is a STATIONS name or an (x, y) point. Returns
    (spec, points, label) for TransportMission / JobQueue; raises ValueError
    with the reason if the job cannot be run."""
    points = {}
    ends = []
    for role, shown, where in (('pickup', 'pick-up', pickup), ('drop_off', 'drop-off', dropoff)):
        if isinstance(where, str):
            if where not in STATIONS:
                raise ValueError(f'unknown station {where!r}')
            ends.append((where, STATIONS[where][:2], where.replace('_', ' ')))
        else:
            x, y = float(where[0]), float(where[1])
            error = validate_goal(x, y)
            if error:
                raise ValueError(f'{shown} {error}')
            points[role] = (x, y)
            ends.append((role, (x, y), f'({x:.1f}, {y:.1f})'))
    (p_name, p_xy, p_label), (d_name, d_xy, d_label) = ends
    if math.hypot(p_xy[0] - d_xy[0], p_xy[1] - d_xy[1]) < MIN_JOB_SEPARATION_M:
        raise ValueError('pick-up and drop-off are the same place')
    if planner is not None:
        if planner.plan(p_xy, d_xy) is None:
            raise ValueError(f'no route from pick-up {p_label} to drop-off {d_label}')
        if planner.plan(d_xy, STATIONS['dock'][:2]) is None:
            raise ValueError(f'no route from drop-off {d_label} back to the dock')
    spec = {'material': 'materials',
            'legs': [(p_name, 'load'), (d_name, 'unload'), ('dock', 'travel')]}
    return spec, points, f'custom: {p_label} → {d_label}'


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
        self.reroutes = 0
        self._last_route_check = now
        self._last_reroute = -float('inf')
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

    def _check_route(self, now, odom_xy, nav_status, station):
        """Reroute around learned obstacles. Returns the new first waypoint,
        or None. Keeps the current route if no alternative exists (the
        no-progress check then fails the leg as usual)."""
        if self.planner is None or not hasattr(self.planner, 'route_blocked'):
            return None
        if now - self._last_route_check < ROUTE_CHECK_S or nav_status in HOLD_STATUSES:
            return None
        self._last_route_check = now
        if now - self._last_reroute < REROUTE_MIN_GAP_S:
            return None
        if not self.planner.route_blocked(odom_xy, self.route):
            return None
        new_route = self.planner.plan(odom_xy, self._xy(station))
        if new_route is None:
            return None
        self.route = new_route
        self.reroutes += 1
        self._last_reroute = now
        # re-baseline the progress measure for the new route, but do NOT
        # reset the no-progress timer: endless reroutes must not hide a stuck robot
        self.best_remaining = self._remaining(odom_xy)
        return self.current_goal()

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
        rerouted = self._check_route(now, odom_xy, nav_status, station)
        if rerouted is not None:
            return rerouted

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
            'reroutes': self.reroutes,
            'no_progress_s': round(max(0.0, now - self.last_progress_t), 1) if self.active else None,
        }


validate_stations()
