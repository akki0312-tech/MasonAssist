import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from safety_supervisor_core import (  # noqa: E402
    CMD_TIMEOUT_S, GUARD_MARGIN_M, LINK_TIMEOUT_S, MAX_ANGULAR, MAX_LINEAR,
    SENSOR_TIMEOUT_S, SafetySupervisorCore,
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
