import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from safety_supervisor_core import (  # noqa: E402
    CMD_TIMEOUT_S, GUARD_MARGIN_M, LINK_TIMEOUT_S, MAX_ANGULAR, MAX_LINEAR,
    SENSOR_TIMEOUT_S, WALL_BACKSTOP_S, SafetySupervisorCore,
)

N = 360
INC = 2 * math.pi / N
A_MIN = -math.pi


def _ranges(hits=None):
    """Empty scan (+inf) with {angle_deg: range} hits, laser frame."""
    r = [math.inf] * N
    for deg, rng in (hits or {}).items():
        r[int(round((math.radians(deg) - A_MIN) / INC)) % N] = rng
    return r


def _healthy(now=0.0, hits=None, link=True):
    """Supervisor with fresh scan, odom, heartbeat and a forward nav command."""
    s = SafetySupervisorCore(require_operator_link=link)
    s.on_scan(_ranges(hits), A_MIN, INC, now)
    s.on_odom(now)
    s.on_heartbeat(now)
    s.on_nav_cmd(0.2, 0.1, now)
    return s


def test_passes_through_when_healthy():
    assert _healthy().output(0.1) == (0.2, 0.1, 'ok', False)


def test_no_command_ever_is_zero():
    s = _healthy()
    s.cmd_t = None
    assert s.output(0.1)[:3] == (0.0, 0.0, 'stale_cmd')


def test_stale_nav_command_stops():
    s = _healthy()
    s.on_scan(_ranges(), A_MIN, INC, 0.3)
    s.on_odom(0.3)
    assert s.output(CMD_TIMEOUT_S - 0.01)[2] == 'ok'
    assert s.output(CMD_TIMEOUT_S + 0.05)[:3] == (0.0, 0.0, 'stale_cmd')


def test_stale_scan_stops():
    s = _healthy()
    s.on_odom(0.6)
    s.on_heartbeat(0.6)
    s.on_nav_cmd(0.2, 0.0, 0.6)
    assert s.output(SENSOR_TIMEOUT_S + 0.1)[:3] == (0.0, 0.0, 'stale_scan')


def test_stale_odom_stops():
    s = _healthy()
    s.on_scan(_ranges(), A_MIN, INC, 0.6)
    s.on_heartbeat(0.6)
    s.on_nav_cmd(0.2, 0.0, 0.6)
    assert s.output(SENSOR_TIMEOUT_S + 0.1)[:3] == (0.0, 0.0, 'stale_odom')


def _refresh_all_but_link(s, now):
    s.on_scan(_ranges(), A_MIN, INC, now)
    s.on_odom(now)
    s.on_nav_cmd(0.2, 0.0, now)


def test_operator_link_lost_stops_and_recovers():
    s = _healthy()
    _refresh_all_but_link(s, LINK_TIMEOUT_S + 0.1)
    assert s.output(LINK_TIMEOUT_S + 0.1)[:3] == (0.0, 0.0, 'link_lost')
    s.on_heartbeat(LINK_TIMEOUT_S + 0.2)
    assert s.output(LINK_TIMEOUT_S + 0.2)[2] == 'ok'


def test_operator_link_not_required():
    s = _healthy(link=False)
    _refresh_all_but_link(s, 10.0)
    assert s.output(10.0)[2] == 'ok'


def test_speed_is_clamped():
    s = _healthy()
    s.on_nav_cmd(1.0, -2.0, 0.0)
    assert s.output(0.1) == (MAX_LINEAR, -MAX_ANGULAR, 'ok', True)
    s.on_nav_cmd(-1.0, 0.0, 0.0)
    assert s.output(0.1) == (-MAX_LINEAR, 0.0, 'ok', True)


def test_estop_latches_until_clear_and_resume():
    s = _healthy()
    s.on_mission_cmd('estop')
    assert s.output(0.1)[:3] == (0.0, 0.0, 'estop')
    s.on_mission_cmd('resume')            # resume alone does not clear an e-stop
    assert s.output(0.1)[2] == 'estop'
    s.on_goal()                           # nor does a new goal
    assert s.output(0.1)[2] == 'estop'
    s.on_mission_cmd('clear_estop')
    assert s.output(0.1)[:3] == (0.0, 0.0, 'hold_after_estop')
    s.on_mission_cmd('resume')
    assert s.output(0.1)[2] == 'ok'


def test_clear_estop_without_estop_does_not_hold():
    s = _healthy()
    s.on_mission_cmd('clear_estop')
    assert s.output(0.1)[2] == 'ok'


def test_cancel_stops_until_new_goal():
    s = _healthy()
    s.on_mission_cmd('cancel')
    assert s.output(0.1)[:3] == (0.0, 0.0, 'cancelled')
    s.on_mission_cmd('resume')
    assert s.output(0.1)[2] == 'cancelled'
    s.on_goal()
    assert s.output(0.1)[2] == 'ok'


def test_collision_guard_blocks_forward_keeps_rotation():
    # body front at x=0.3625; lidar at x=0.30 -> a hit 0.1 m past the front
    s = _healthy(hits={0: 0.3625 - 0.30 + 0.1})
    assert s.front_clear < GUARD_MARGIN_M
    assert s.output(0.1) == (0.0, 0.1, 'collision_guard', False)
    s.on_nav_cmd(-0.1, 0.3, 0.0)          # reversing away is allowed
    assert s.output(0.1) == (-0.1, 0.3, 'ok', False)


def test_collision_guard_blocks_reverse():
    # straight behind: lidar->x = 0.30 - r; body rear at x=-0.3625
    s = _healthy(hits={180: 0.30 + 0.3625 + 0.05})
    s.on_nav_cmd(-0.1, 0.0, 0.0)
    assert s.output(0.1)[2] == 'collision_guard'
    s.on_nav_cmd(0.1, 0.0, 0.0)
    assert s.output(0.1)[2] == 'ok'


def test_obstacle_beyond_margin_or_outside_lane_is_ignored():
    s = _healthy(hits={0: 0.3625 - 0.30 + 0.5})          # 0.5 m ahead
    assert s.output(0.1)[2] == 'ok'
    s = _healthy(hits={60: 0.6})                          # y = 0.52 > half width
    assert s.output(0.1)[2] == 'ok'


def test_self_occlusion_bands_ignored():
    # the robot's own tracks show up near +-100 deg at short range
    s = _healthy(hits={100: 0.3, -100: 0.3})
    assert s.output(0.1)[2] == 'ok'


def test_nonfinite_and_too_close_returns_ignored():
    s = _healthy(hits={0: float('nan'), 2: -math.inf, 4: 0.05})
    assert s.front_clear == math.inf
    assert s.output(0.1)[2] == 'ok'


def test_estop_has_priority_over_stale_data():
    s = SafetySupervisorCore()
    s.on_mission_cmd('estop')
    assert s.output(100.0)[2] == 'estop'


def _healthy_sim(wall, sim):
    s = SafetySupervisorCore()
    s.on_scan(_ranges(), A_MIN, INC, wall, sim)
    s.on_odom(wall, sim)
    s.on_heartbeat(wall)
    s.on_nav_cmd(0.2, 0.0, wall, sim)
    return s


def test_simulation_hiccup_is_not_a_stale_command():
    # measured with the Gazebo GUI: 0.35 s of wall time passed but only
    # 0.15 s of sim time - the robot could not have moved further
    s = _healthy_sim(10.0, 5.0)
    assert s.output(10.35, 5.15)[2] == 'ok'


def test_stale_command_measured_in_sim_time():
    s = _healthy_sim(10.0, 5.0)
    s.on_scan(_ranges(), A_MIN, INC, 10.4, 5.4)
    s.on_odom(10.4, 5.4)
    assert s.output(10.4, 5.0 + CMD_TIMEOUT_S + 0.05)[2] == 'stale_cmd'


def test_frozen_sim_clock_still_stops_via_wall_backstop():
    # sim time not advancing (frozen /clock): wall time still bounds the age
    s = _healthy_sim(10.0, 5.0)
    assert s.output(10.0 + WALL_BACKSTOP_S - 0.05, 5.0)[2] == 'ok'
    assert s.output(10.0 + WALL_BACKSTOP_S + 0.05, 5.0)[2] == 'stale_scan'


def test_operator_link_uses_wall_time_only():
    s = _healthy_sim(10.0, 5.0)
    s.on_scan(_ranges(), A_MIN, INC, 10.0 + LINK_TIMEOUT_S + 0.1, 5.0)
    s.on_odom(10.0 + LINK_TIMEOUT_S + 0.1, 5.0)
    s.on_nav_cmd(0.2, 0.0, 10.0 + LINK_TIMEOUT_S + 0.1, 5.0)
    assert s.output(10.0 + LINK_TIMEOUT_S + 0.1, 5.0)[2] == 'link_lost'


# ---------------------------------------------------------------------------
# tilt guard (IMU)
# ---------------------------------------------------------------------------

from safety_supervisor_core import (  # noqa: E402
    TILT_RELEASE_DEG, TILT_SLOW_BAND_DEG, TILT_SLOW_SPEED, tilt_from_quaternion, tilt_limit,
)


def _q_pitch(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def _q_roll(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def test_tilt_from_quaternion():
    assert tilt_from_quaternion(0, 0, 0, 1) == pytest.approx(0.0)
    assert tilt_from_quaternion(*_q_pitch(-20)) == pytest.approx(20.0)
    assert tilt_from_quaternion(*_q_roll(10)) == pytest.approx(10.0)
    h = math.radians(90) / 2                    # heading alone is not tilt
    assert tilt_from_quaternion(0, 0, math.sin(h), math.cos(h)) == pytest.approx(0.0)


def test_tilt_limit_by_payload():
    assert tilt_limit(0) == tilt_limit(25) == 35.0
    assert tilt_limit(50) == tilt_limit(80) == 22.0
    assert tilt_limit(37.5) == pytest.approx(28.5)


def _tilted(deg, payload=50.0, cmd=0.2):
    s = SafetySupervisorCore(use_imu=True, payload_kg=payload)
    s.on_scan(_ranges(), A_MIN, INC, 0.0)
    s.on_odom(0.0)
    s.on_heartbeat(0.0)
    s.on_imu(*_q_pitch(-deg), 0.0)
    s.on_nav_cmd(cmd, 0.1, 0.0)
    return s


def test_level_robot_unaffected():
    assert _tilted(0.0).output(0.1) == (0.2, 0.1, 'ok', False)


def test_slow_band_clamps_linear_speed():
    limit = tilt_limit(50)
    s = _tilted(limit - TILT_SLOW_BAND_DEG + 1, cmd=0.25)
    assert s.output(0.1) == (TILT_SLOW_SPEED, 0.1, 'ok', True)


def test_at_the_limit_stop_then_release_with_hysteresis():
    limit = tilt_limit(50)
    s = _tilted(limit + 0.5)
    assert s.output(0.1)[:3] == (0.0, 0.0, 'tilt_limit')
    s.on_imu(*_q_pitch(-(limit - TILT_RELEASE_DEG / 2)), 0.05)    # inside the hysteresis
    assert s.output(0.1)[2] == 'tilt_limit'
    s.on_imu(*_q_pitch(-(limit - TILT_RELEASE_DEG - 0.5)), 0.05)  # below it: released
    assert s.output(0.1)[2] == 'ok'


def test_same_tilt_is_fine_without_payload():
    assert _tilted(30.0, payload=0.0, cmd=0.2).output(0.1)[2] == 'ok'
    assert _tilted(30.0, payload=50.0).output(0.1)[2] == 'tilt_limit'


def test_resume_allows_creeping_off_the_slope():
    s = _tilted(25.0)
    assert s.output(0.1)[2] == 'tilt_limit'
    s.on_mission_cmd('resume')
    s.on_nav_cmd(-0.25, 0.0, 0.05)
    assert s.output(0.1) == (-TILT_SLOW_SPEED, 0.0, 'ok', True)
    s.on_imu(*_q_pitch(-5.0), 0.08)               # off the slope: back to normal
    s.on_nav_cmd(0.2, 0.0, 0.08)
    assert s.output(0.1)[:3] == (0.2, 0.0, 'ok')


def test_stale_imu_stops_when_imu_is_used():
    s = _tilted(0.0)
    s.on_scan(_ranges(), A_MIN, INC, 0.6)
    s.on_odom(0.6)
    s.on_heartbeat(0.6)
    s.on_nav_cmd(0.2, 0.0, 0.6)
    assert s.output(0.6)[:3] == (0.0, 0.0, 'stale_imu')
    assert SafetySupervisorCore().use_imu is False      # off unless enabled (older setups)


# ---------------------------------------------------------------------------
# no-go zones
# ---------------------------------------------------------------------------

from safety_supervisor_core import BODY_FRONT_M, BODY_REAR_M, NO_GO_LOOKAHEAD_M  # noqa: E402

ZONE = {'steep_descent': (5.0, 0.0, 1.0, 1.0)}          # x 4..6, y -1..1


def _zoned(x, y, yaw, cmd=0.2, zones=ZONE):
    s = SafetySupervisorCore(zones=zones)
    s.on_scan(_ranges(), A_MIN, INC, 0.0)
    s.on_odom(0.0)
    s.on_heartbeat(0.0)
    s.on_pose(x, y, yaw, 0.0)
    s.on_nav_cmd(cmd, 0.1, 0.0)
    return s


def test_driving_toward_a_zone_is_stopped_before_it():
    # front edge at x = 4 - lookahead/2: inside the lookahead
    x = 4.0 - BODY_FRONT_M - NO_GO_LOOKAHEAD_M / 2
    s = _zoned(x, 0.0, 0.0)
    assert s.output(0.1) == (0.0, 0.1, 'no_go_zone', False) and s.zone_hit == 'steep_descent'
    # further away: fine
    assert _zoned(x - 1.0, 0.0, 0.0).output(0.1)[2] == 'ok'


def test_reversing_into_a_zone_is_stopped_and_away_is_allowed():
    x = 4.0 - BODY_REAR_M - NO_GO_LOOKAHEAD_M / 2          # facing away (yaw pi), backing toward it
    assert _zoned(x, 0.0, math.pi, cmd=-0.2).output(0.1)[2] == 'no_go_zone'
    assert _zoned(x, 0.0, math.pi, cmd=0.2).output(0.1)[2] == 'ok'
    assert _zoned(x, 0.0, 0.0, cmd=-0.2).output(0.1)[2] == 'ok'   # backing away from it


def test_inside_a_zone_only_creeping_out_is_allowed():
    s = _zoned(4.3, 0.0, 0.0, cmd=0.25)                     # inside near the west edge, facing in
    assert s.output(0.1)[2] == 'no_go_zone'                 # deeper: no
    s.on_nav_cmd(-0.25, 0.0, 0.0)
    assert s.output(0.1) == (-TILT_SLOW_SPEED, 0.0, 'ok', True)   # back out, at creep speed


def test_no_zones_or_stale_pose():
    assert _zoned(3.0, 0.0, 0.0, zones={}).output(0.1)[2] == 'ok'
    s = _zoned(0.0, 0.0, 0.0)
    s.on_scan(_ranges(), A_MIN, INC, 0.6)
    s.on_odom(0.6)
    s.on_heartbeat(0.6)
    s.on_nav_cmd(0.2, 0.0, 0.6)
    assert s.output(0.6)[2] == 'stale_pose'


def test_zone_payload_filtering():
    from site_config import zones_blocking
    zones = {'a': {'box': ZONE['steep_descent'], 'max_payload_kg': 25.0, 'reason': ''}}
    assert zones_blocking(zones, 0.0) == {} and zones_blocking(zones, 25.0) == {}
    assert zones_blocking(zones, 50.0) == {'a': ZONE['steep_descent']}
