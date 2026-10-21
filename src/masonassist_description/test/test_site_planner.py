import math
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
