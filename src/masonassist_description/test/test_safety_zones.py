import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from safety_zones import (  # noqa: E402
    RESUME_CLEAR_S, SLOW_FACTOR, SafetyGate, classify, unmapped_points,
)


def test_known_wall_returns_are_mapped_and_ignored():
    # robot at (0, 6) facing north; wall_north face is at y = 7.8 -> 1.8 m ahead
    r, a = unmapped_points([1.8, 1.8], [0.0, 0.1], (0.0, 6.0, math.pi / 2))
    assert r.size == 0


def test_return_in_open_space_is_unmapped():
    # robot at dock facing west; point 2 m ahead at (-2, 0) is open floor
    r, a = unmapped_points([2.0], [0.0], (0.0, 0.0, math.pi))
    assert r.size == 1


def test_stop_when_unmapped_point_close_ahead():
    zone, d = classify(0.2, 0.0, [0.8], [0.0])
    assert zone == 'stop' and abs(d - 0.8) < 1e-9


def test_slow_when_unmapped_point_within_slow_range():
    assert classify(0.2, 0.0, [1.5], [0.05])[0] == 'slow'


def test_point_beside_path_does_not_count():
    assert classify(0.2, 0.0, [1.5], [math.radians(90)])[0] == 'clear'


def test_reversing_checks_the_rear_not_the_front():
    assert classify(-0.08, 0.0, [0.8], [0.0])[0] == 'clear'
    assert classify(-0.08, 0.0, [0.8], [math.pi])[0] == 'stop'


def test_turning_in_place_stops_only_for_very_close_points():
    assert classify(0.0, 0.3, [0.7], [math.radians(120)])[0] == 'stop'
    assert classify(0.0, 0.3, [1.2], [math.radians(120)])[0] == 'clear'


def test_gate_scales_linear_in_slow_zone_and_keeps_turning():
    lin, ang, zone, _ = SafetyGate().filter(0.2, 0.1, np.array([1.5]), np.array([0.0]), now=0.0)
    assert zone == 'slow' and abs(lin - 0.2 * SLOW_FACTOR) < 1e-9 and ang == 0.1


def test_gate_holds_stop_until_clear_for_resume_delay():
    g = SafetyGate()
    assert g.filter(0.2, 0.0, np.array([0.8]), np.array([0.0]), 0.0)[2] == 'stop'
    empty = np.array([])
    lin, _, zone, _ = g.filter(0.2, 0.0, empty, empty, 0.2)
    assert zone == 'stop' and lin == 0.0                       # cleared, but not long enough
    lin, _, zone, _ = g.filter(0.2, 0.0, empty, empty, 0.2 + RESUME_CLEAR_S + 0.01)
    assert zone == 'clear' and lin == 0.2


def test_stopped_robot_may_still_turn_if_nothing_is_close_on_any_side():
    g = SafetyGate()
    # person 0.9 m dead ahead: stop forward motion, but turning is safe (> 0.8 m)
    lin, ang, zone, _ = g.filter(0.2, 0.3, np.array([0.9]), np.array([0.0]), 0.0)
    assert zone == 'stop' and lin == 0.0 and ang == 0.3
    # person 0.5 m away: no turning either
    lin, ang, zone, _ = g.filter(0.2, 0.3, np.array([0.5]), np.array([0.0]), 0.1)
    assert zone == 'stop' and lin == 0.0 and ang == 0.0
