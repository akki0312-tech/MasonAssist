import math

import numpy as np
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import pytest  # noqa: E402

from site_planner import INFLATION_M, SitePlanner, _blocked  # noqa: E402
from transport_mission import MISSIONS, STATIONS  # noqa: E402


@pytest.fixture(scope='module')
def planner():
    return SitePlanner()


def _segment_clear(p, a, b, inflation):
    steps = max(int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05), 1)
    return all(not _blocked(a[0] + (b[0] - a[0]) * k / steps,
                            a[1] + (b[1] - a[1]) * k / steps, inflation)
               for k in range(steps + 1))


def test_every_mission_leg_has_a_route_that_keeps_clearance(planner):
    for spec in MISSIONS.values():
        pos = STATIONS['dock'][:2]
        for station, _ in spec['legs']:
            goal = STATIONS[station][:2]
            route = planner.plan(pos, goal)
            assert route is not None, station
            assert route[-1] == goal
            prev = pos
            for wp in route:
                # small tolerance for grid discretisation
                assert _segment_clear(planner, prev, wp, INFLATION_M - 0.12), (station, prev, wp)
                prev = wp
            pos = goal


def test_route_leaves_corridor_before_turning_toward_storage(planner):
    # straight line dock -> brick_pickup clips partition_b's west corner
    route = planner.plan((0.0, 0.0), STATIONS['brick_pickup'][:2])
    assert len(route) >= 2
    assert route[0][0] < -1.6   # first waypoint is past the partition ends


def test_unreachable_goal_inside_wall_still_ends_at_requested_goal_or_none(planner):
    route = planner.plan((0.0, 0.0), (0.0, 8.0))
    assert route is None or route[-1] == (0.0, 8.0)


def _blob_mask(planner, cx, cy, radius):
    xs = planner.lo + np.arange(planner.n) * planner.res
    gx, gy = np.meshgrid(xs, xs, indexing='ij')
    return np.hypot(gx - cx, gy - cy) <= radius


def _min_dist_along(start, route, cx, cy, step=0.05):
    pts, best = [tuple(start)] + list(route), math.inf
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = max(int(math.hypot(x1 - x0, y1 - y0) / step), 1)
        for k in range(n + 1):
            best = min(best, math.hypot(x0 + (x1 - x0) * k / n - cx, y0 + (y1 - y0) * k / n - cy))
    return best


def test_learned_obstacle_is_routed_around(planner):
    start, goal = (0.0, 0.0), STATIONS['brick_pickup'][:2]
    old = planner.plan(start, goal)
    try:
        planner.set_dynamic(_blob_mask(planner, -3.9, -2.4, 0.5))
        assert planner.route_blocked(start, old)
        new = planner.plan(start, goal)
        assert new is not None and new[-1] == goal
        assert not planner.route_blocked(start, new)
        assert _min_dist_along(start, new, -3.9, -2.4) >= 0.5 + 0.5   # blob + clearance
    finally:
        planner.set_dynamic(None)
    assert planner.plan(start, goal) == old        # no learned map -> unchanged behaviour
