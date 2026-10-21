import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from transport_mission import (  # noqa: E402
    ARRIVAL_TOLERANCE_M, DWELL_S, MIN_STATION_CLEARANCE_M, MISSIONS, STATIONS,
    TransportMission, station_clearance,
)


def _at(name):
    x, y, _ = STATIONS[name]
    return (x, y)


def test_all_stations_clear_of_obstacles():
    for name, (x, y, _) in STATIONS.items():
        assert station_clearance(x, y) >= MIN_STATION_CLEARANCE_M, name


def test_mission_legs_reference_known_stations():
    for spec in MISSIONS.values():
        for station, action in spec['legs']:
            assert station in STATIONS
            assert action in ('travel', 'load', 'unload')


def test_first_goal_is_first_leg_station():
    m = TransportMission('bricks_to_building', now=0.0)
    assert m.current_goal() == _at('brick_pickup')
    assert m.phase == 'traveling' and m.payload == 'empty'


def test_stale_reached_far_from_station_does_not_advance():
    m = TransportMission('bricks_to_building', now=0.0)
    assert m.update(1.0, (0.0, 0.0), 'reached') is None
    assert m.leg_index == 0 and m.phase == 'traveling'


def test_not_reached_status_does_not_advance_even_at_station():
    m = TransportMission('bricks_to_building', now=0.0)
    assert m.update(1.0, _at('brick_pickup'), 'navigating') is None
    assert m.phase == 'traveling'


def test_full_mission_load_unload_and_complete():
    m = TransportMission('bricks_to_building', now=0.0)

    assert m.update(10.0, _at('brick_pickup'), 'reached') is None
    assert m.phase == 'loading' and m.payload == 'empty'
    assert m.update(10.0 + DWELL_S / 2, _at('brick_pickup'), 'reached') is None
    goal = m.update(10.0 + DWELL_S + 0.1, _at('brick_pickup'), 'reached')
    assert goal == _at('building_door')
    assert m.payload == 'bricks' and m.phase == 'traveling' and m.leg_index == 1

    # stale 'reached' at the pickup must not complete the delivery leg
    assert m.update(14.0, _at('brick_pickup'), 'reached') is None
    assert m.leg_index == 1

    m.update(30.0, _at('building_door'), 'reached')
    assert m.phase == 'unloading'
    goal = m.update(30.0 + DWELL_S + 0.1, _at('building_door'), 'reached')
    assert goal == _at('dock') and m.payload == 'empty'

    x, y = _at('dock')
    assert m.update(50.0, (x + ARRIVAL_TOLERANCE_M * 0.5, y), 'reached') is None
    assert m.phase == 'complete' and not m.active
    assert m.to_dict(60.0)['elapsed_s'] == 50.0


def test_abort_stops_further_goals():
    m = TransportMission('blocks_to_scaffold', now=0.0)
    m.abort('cancelled', now=5.0)
    assert m.phase == 'aborted' and m.abort_reason == 'cancelled'
    assert m.current_goal() is None
    assert m.update(6.0, _at('block_pickup'), 'reached') is None


def test_planned_mission_walks_waypoints_then_station():
    from site_planner import SitePlanner
    m = TransportMission('bricks_to_building', now=0.0, start_xy=(0.0, 0.0), planner=SitePlanner())
    route = list(m.route)
    assert len(route) >= 2 and route[-1] == _at('brick_pickup')
    assert m.current_goal() == route[0]
    # reaching intermediate waypoints publishes later ones without needing 'reached'
    for i in range(len(route)):
        if m.current_goal() == route[-1]:
            break
        assert m.update(1.0 + i, m.current_goal(), 'navigating') in route
    # a stale 'reached' far from the station never completes the leg
    assert m.update(20.0, route[-2], 'reached') is None and m.leg_index == 0
    m.update(21.0, _at('brick_pickup'), 'reached')
    assert m.phase == 'loading'


def test_validate_goal_rejects_outside_site_and_inside_obstacles():
    from transport_mission import validate_goal
    assert validate_goal(20.0, 0.0) is not None          # beyond perimeter
    assert validate_goal(0.0, 8.0) is not None            # inside wall_north
    assert validate_goal(2.0, -3.0) is not None           # inside barrier_1
    assert validate_goal(-3.0, -3.0) is None              # open ground
    assert validate_goal(*_at('dock')) is None


def test_goto_mission_routes_behind_partition_and_completes_without_dwell():
    from site_planner import SitePlanner
    m = TransportMission.goto(-3.0, -3.0, now=0.0, start_xy=(0.0, 0.0), planner=SitePlanner())
    assert m.name == 'goto' and m.active
    route = list(m.route)
    assert len(route) >= 2 and route[-1] == (-3.0, -3.0)
    # drive to whatever goal is current; waypoints already within reach are skipped
    for i in range(len(route)):
        if m.current_goal() == route[-1]:
            break
        nxt = m.update(1.0 + i, m.current_goal(), 'navigating')
        assert nxt in route
    assert m.current_goal() == (-3.0, -3.0)
    assert m.update(30.0, (-3.0 + 0.1, -3.0), 'reached') is None
    assert m.phase == 'complete' and m.payload == 'empty'
    d = m.to_dict(31.0)
    assert d['legs'] == [{'station': 'goal', 'action': 'travel', 'x': -3.0, 'y': -3.0}]
