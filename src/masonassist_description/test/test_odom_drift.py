import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from odom_drift_core import OdomDrift  # noqa: E402


def _drive(d, poses):
    out = None
    for p in poses:
        out = d.update(*p)
    return out


def test_first_pose_is_passed_through_and_stillness_never_drifts():
    d = OdomDrift()
    assert d.update(1.0, 2.0, 0.3) == (1.0, 2.0, 0.3)
    for _ in range(1000):
        assert d.update(1.0, 2.0, 0.3) == (1.0, 2.0, 0.3)


def test_zero_errors_reproduce_the_truth():
    d = OdomDrift(0.0, 0.0, 0.0)
    path = [(0.01 * i, 0.002 * i * i / 100, 0.01 * i) for i in range(500)]
    x, y, yaw = _drive(d, path)
    assert (x, y) == pytest.approx(path[-1][:2], abs=1e-9)
    assert math.atan2(math.sin(yaw - path[-1][2]), math.cos(yaw - path[-1][2])) == pytest.approx(0.0, abs=1e-9)


def test_straight_line_gets_the_linear_scale_error():
    d = OdomDrift(linear_scale_err=0.03, yaw_scale_err=0.0, yaw_bias_walk=0.0)
    x, y, yaw = _drive(d, [(0.01 * i, 0.0, 0.0) for i in range(1001)])   # 10 m
    assert x == pytest.approx(10.3) and y == pytest.approx(0.0) and yaw == 0.0


def test_turning_in_place_gets_the_yaw_scale_error():
    d = OdomDrift(linear_scale_err=0.0, yaw_scale_err=0.06, yaw_bias_walk=0.0)
    d.update(0.0, 0.0, 0.0)
    yaw = 0.0
    for i in range(1, 101):                       # +90 deg in small steps
        _, _, yaw = d.update(0.0, 0.0, math.radians(0.9 * i))
    assert math.degrees(yaw) == pytest.approx(90 * 1.06)


def test_same_seed_same_drift_different_seed_different_drift():
    path = [(0.02 * i, 0.0, 0.0) for i in range(500)]
    a = _drive(OdomDrift(seed=7), path)
    b = _drive(OdomDrift(seed=7), path)
    c = _drive(OdomDrift(seed=8), path)
    assert a == b and a != c


def test_heading_bias_bends_a_straight_drive():
    d = OdomDrift(linear_scale_err=0.0, yaw_scale_err=0.0, yaw_bias_walk=0.05, seed=3)
    _, y, yaw = _drive(d, [(0.01 * i, 0.0, 0.0) for i in range(2001)])   # 20 m straight
    assert abs(yaw) > 0.01 and abs(y) > 0.05


def test_twist_scaled_like_the_pose():
    d = OdomDrift(linear_scale_err=0.03, yaw_scale_err=0.06, yaw_bias_walk=0.0)
    assert d.twist(0.2, 0.5) == pytest.approx((0.206, 0.53))


def test_compose_se2_map_odom_base():
    from odom_drift_core import compose_se2
    # odom frame shifted 1 m in x and turned 90 deg in the map
    assert compose_se2((1.0, 0.0, math.pi / 2), (2.0, 0.0, 0.0)) == pytest.approx((1.0, 2.0, math.pi / 2))
    assert compose_se2((0.0, 0.0, 0.0), (3.0, -1.0, 0.4)) == pytest.approx((3.0, -1.0, 0.4))
    x, y, th = compose_se2((0.5, 0.5, math.pi), (0.0, 0.0, math.pi / 2))
    assert (x, y) == pytest.approx((0.5, 0.5)) and th == pytest.approx(-math.pi / 2)
