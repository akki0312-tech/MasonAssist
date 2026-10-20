#!/usr/bin/env python3
"""Unit tests for scripts/vfh_plus_core.py - the standalone VFH+ core.

All scans here are synthetic and sparse (only the points relevant to each
scenario): points beyond d_max never contribute anything (see
build_raw_histogram), so omitting an explicit "background" at max range is
equivalent to including one and simpler to reason about.

Run with: python3 -m pytest test/test_vfh_plus_core.py -v
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from vfh_plus_core import (  # noqa: E402
    VFHParams,
    VFHPlusCore,
    VFHState,
    Valley,
    build_raw_histogram,
    enlargement_half_angle,
    is_direction_safe,
    point_magnitude,
    sectors_in_span,
    wrap_to_pi,
)


def deg(d):
    return math.radians(d)


def default_params():
    return VFHParams()


# ---------------------------------------------------------------------------
# A. Empty environment
# ---------------------------------------------------------------------------

def test_a_empty_environment():
    core = VFHPlusCore(default_params())
    ranges = np.array([core.params.range_max] * 8)
    angles = np.linspace(-math.pi, math.pi, 8, endpoint=False)
    result = core.process(ranges, angles)

    assert result.state == VFHState.OK
    assert not result.binary_histogram.any()
    assert result.selected_direction is not None
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


# ---------------------------------------------------------------------------
# B. Single obstacle directly ahead
# ---------------------------------------------------------------------------

def test_b_obstacle_directly_ahead():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[2.0], angles=[0.0])

    assert result.state == VFHState.OK
    assert result.binary_histogram[0]  # sector centered at 0 rad is occupied
    assert result.selected_direction is not None
    assert abs(result.selected_direction) > 1e-6  # steered away from 0

    # Determinism: a second, fresh instance with the same input agrees.
    core2 = VFHPlusCore(default_params())
    result2 = core2.process(ranges=[2.0], angles=[0.0])
    assert result2.selected_direction == pytest.approx(result.selected_direction)


# ---------------------------------------------------------------------------
# C. Obstacle directly ahead, close range
# ---------------------------------------------------------------------------

def test_c_obstacle_close_range():
    core_far = VFHPlusCore(default_params())
    result_far = core_far.process(ranges=[2.0], angles=[0.0])

    core_close = VFHPlusCore(default_params())
    result_close = core_close.process(ranges=[0.3], angles=[0.0])

    occupied_far = int(result_far.binary_histogram.sum())
    occupied_close = int(result_close.binary_histogram.sum())
    assert occupied_close > occupied_far

    # Spec explicitly anticipates either outcome for a very close obstacle.
    if result_close.state == VFHState.OK:
        assert abs(wrap_to_pi(result_close.selected_direction)) > deg(45)
    else:
        assert result_close.state == VFHState.BLOCKED


# ---------------------------------------------------------------------------
# D / E. Obstacle on left / right
# ---------------------------------------------------------------------------

def test_d_obstacle_on_left_steers_right():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.2], angles=[deg(20)])
    assert result.state == VFHState.OK
    assert result.selected_direction < 0.0


def test_e_obstacle_on_right_steers_left():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.2], angles=[deg(-20)])
    assert result.state == VFHState.OK
    assert result.selected_direction > 0.0


# ---------------------------------------------------------------------------
# F. Corner-contact regression (THE important test)
# ---------------------------------------------------------------------------

def test_f_corner_contact_regression():
    """Reproduces the geometry of the observed failure: a point at
    base_link-frame (x=0.35, y=0.30) is d=0.461 m from base_link (inside
    r_effective=0.5975 m) at bearing 40.6 deg - well outside the OLD
    controller's +-15 deg (0.26 rad) front cone, so the old stateless
    algorithm would have called this "clear ahead." This test asserts the
    new footprint-aware histogram correctly marks straight-ahead (0 rad) as
    occupied anyway."""
    x, y = 0.35, 0.30
    d = math.hypot(x, y)
    angle = math.atan2(y, x)
    assert d == pytest.approx(0.4610, abs=1e-3)
    assert math.degrees(angle) == pytest.approx(40.6, abs=0.5)

    old_front_half_angle = deg(15)  # simple_obstacle_avoidance.py's front_half_angle
    assert angle > old_front_half_angle  # confirms: outside the old cone

    core = VFHPlusCore(default_params())
    result = core.process(ranges=[d], angles=[angle])

    assert result.binary_histogram[0], (
        "footprint inflation failed to mark straight-ahead unsafe for an "
        "obstacle that would graze the chassis - this is the regression "
        "this module exists to prevent"
    )
    if result.state == VFHState.OK:
        assert abs(wrap_to_pi(result.selected_direction)) > old_front_half_angle


# ---------------------------------------------------------------------------
# G. Corridor (wider than the robot)
# ---------------------------------------------------------------------------

def test_g_corridor_wider_than_robot():
    # Closest approach of each wall: (d=1.0, angle=+-90 deg).
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.0, 1.0], angles=[deg(90), deg(-90)])

    assert result.state == VFHState.OK
    assert not result.binary_histogram[0]
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


# ---------------------------------------------------------------------------
# H / I. Isolated two-point geometry: wide enough / genuinely too tight
# ---------------------------------------------------------------------------

def test_h1_isolated_points_wide_enough_accepted():
    """Two obstacles at +-45 deg, d=1.2 m. An EARLIER version of this test
    asserted this gap should be REJECTED, using the old required-width
    heuristic (angular gap ~30 deg vs a computed "required" ~59.7 deg).
    That heuristic was mathematically wrong: the actual perpendicular
    clearance from either point to the straight-ahead (0 rad) heading is
    d*sin(45deg) = 1.2*0.7071 = 0.849 m, comfortably more than
    r_effective = 0.5975 m (0.27 m to spare). Straight ahead is genuinely
    safe, and the new geometric check now correctly accepts it."""
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.2, 1.2], angles=[deg(45), deg(-45)])

    assert result.state == VFHState.OK
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


def test_h2_genuinely_tight_two_point_geometry_rejected():
    """Two obstacles at +-10 deg, d=1.2 m - close enough in angle that their
    enlargement spans (gamma=29.86 deg each) overlap and merge straight
    through 0 rad. This is a genuinely too-tight gap: 0 rad itself must
    never be an accepted candidate.

    UPDATED per the multi-candidate design review/approval: this test
    never actually asserted a global BLOCKED state (it only asserted
    sector 0's occupancy and that no accepted valley's SPAN contains
    sector 0, which remains true regardless of candidate-selection
    changes, since sector 0's occupied notch is never part of any free
    run). What DOES change under multi-candidate selection is that the
    robot may now find a safe exterior detour around both points, exactly
    as approved for the re-scoped F/narrow-gap criterion - confirmed
    directly below via is_direction_safe on the actual selected direction,
    rather than assuming the state stays BLOCKED."""
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.2, 1.2], angles=[deg(10), deg(-10)])

    assert result.binary_histogram[0]
    for v in result.candidate_valleys:
        assert not _valley_contains_sector(v, 0, core.params.num_sectors), (
            "a genuinely too-tight gap must not be an accepted candidate at 0 rad"
        )
    # Whatever the resulting state is, the selected direction (if any) must
    # never thread the gap unsafely - the actual safety property that
    # matters, independent of whether a detour exists.
    if result.state == VFHState.OK:
        assert is_direction_safe(
            result.selected_direction, [1.2, 1.2], [deg(10), deg(-10)],
            core.params.r_effective, core.params.d_max,
        )


def test_i_sufficient_gap_accepted():
    # Two obstacles at +-70 deg, d=1.2 m: actual free gap ~80 deg, required
    # ~59.7 deg - gap is wide enough.
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.2, 1.2], angles=[deg(70), deg(-70)])

    assert result.state == VFHState.OK
    assert any(_valley_contains_sector(v, 0, core.params.num_sectors) for v in result.candidate_valleys)
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


def _valley_contains_sector(v: Valley, sector: int, n: int) -> bool:
    span = (v.end_sector - v.start_sector) % n + 1
    rel = (sector - v.start_sector) % n
    return rel < span


def _wall_points(xs, y):
    """Points along a wall at constant lateral offset y, sampled at each x
    in xs, returned as (ranges, angles) in robot-frame polar coordinates."""
    ranges = [math.hypot(x, y) for x in xs]
    angles = [math.atan2(y, x) for x in xs]
    return ranges, angles


# ---------------------------------------------------------------------------
# Corridor: result must not depend on how far the walls are sampled
# ---------------------------------------------------------------------------

def test_corridor_short_wall_sampling():
    # Closest approach of each wall only: (d=1.0, angle=+-90 deg).
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.0, 1.0], angles=[deg(90), deg(-90)])

    assert result.state == VFHState.OK
    assert not result.binary_histogram[0]
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


def test_corridor_long_wall_sampling_same_result():
    """Same corridor (walls at y=+-1.0 m), but also including a far, nearly-
    tangential point per wall (x=3: d=3.162 m, angle=18.43 deg) - exactly
    the kind of point the OLD required-width heuristic could pick as "the"
    bordering obstacle and get a sampling-dependent wrong answer from. The
    result must agree with test_corridor_short_wall_sampling.

    Sampling is deliberately sparse (x in {0, 3} per wall), not a densely
    resampled wall: with many closely-spaced points, cumulative overlap
    between their individually-widened occupancy spans (see the `ceil` in
    `sectors_in_span`) can push a *neighboring* sector's smoothed magnitude
    over threshold_high, which the 3-sector smoothing window then leaks
    into sector 0 even though no single point's span reaches it and the
    direct geometric check (`is_direction_safe`) independently confirms
    0 rad is safe. That is a separate, real interaction between smoothing
    and dense-wall sampling - not something this task's fix addresses (see
    the implementation report) - so this test targets the specific,
    original sampling-extent bug without also tripping over it."""
    xs = [0, 3]
    left_r, left_a = _wall_points(xs, 1.0)
    right_r, right_a = _wall_points(xs, -1.0)

    core = VFHPlusCore(default_params())
    result = core.process(ranges=left_r + right_r, angles=left_a + right_a)

    assert result.state == VFHState.OK
    assert not result.binary_histogram[0]
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


# ---------------------------------------------------------------------------
# Angled wall: outcome governed by the actually-closest point, not by
# whichever sample happens to border a histogram sector
# ---------------------------------------------------------------------------

def test_angled_wall_safe_margin_from_near_point():
    # Wall points at (x=1, y=0.8) and (x=3, y=1.5): the near point's lateral
    # offset (0.8 m) exceeds r_effective (0.5975 m), so straight ahead clears.
    pts = [(1, 0.8), (3, 1.5)]
    ranges = [math.hypot(x, y) for x, y in pts]
    angles = [math.atan2(y, x) for x, y in pts]

    core = VFHPlusCore(default_params())
    result = core.process(ranges=ranges, angles=angles)

    assert result.state == VFHState.OK
    assert not result.binary_histogram[0]
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


def test_angled_wall_near_point_governs_rejection():
    # Same far point, but the near point moved to y=0.5 - inside
    # r_effective (0.5975 m). Straight ahead must now be rejected, driven
    # by the near point; the far, more-tangential point is irrelevant.
    pts = [(1, 0.5), (3, 1.5)]
    ranges = [math.hypot(x, y) for x, y in pts]
    angles = [math.atan2(y, x) for x, y in pts]

    core = VFHPlusCore(default_params())
    result = core.process(ranges=ranges, angles=angles)

    assert result.binary_histogram[0]
    for v in result.candidate_valleys:
        assert not _valley_contains_sector(v, 0, core.params.num_sectors)


# ---------------------------------------------------------------------------
# Hysteresis-staleness: the geometric gate must catch what a stale "free"
# binary classification misses
# ---------------------------------------------------------------------------

def test_hysteresis_staleness_caught_by_geometric_gate():
    """Establish sector 0 as free (no obstacle). Then, in the very next
    call, place a point at (d=3.0, angle=0) whose magnitude (0.25) falls
    inside the hysteresis dead-band (0.2, 0.4) - the binary histogram
    RETAINS free at sector 0 by design. But this point, evaluated fresh by
    the geometric gate, is unsafe for candidate=0 (lateral=0 < r_effective).
    The gate must reject it despite the stale-free classification."""
    params = default_params()
    core = VFHPlusCore(params)

    core.process(ranges=[params.range_max], angles=[0.0])
    assert not core.binary_state[0]

    result = core.process(ranges=[3.0], angles=[0.0])
    assert params.threshold_low < result.histogram[0] < params.threshold_high, (
        "test construction assumption: call 2's smoothed magnitude should "
        f"land inside the hysteresis band, got {result.histogram[0]}"
    )
    assert not core.binary_state[0], "hysteresis should retain the free classification"
    assert not is_direction_safe(0.0, [3.0], [0.0], params.r_effective, params.d_max), (
        "sanity check: this point must be geometrically unsafe for candidate=0"
    )
    assert result.state != VFHState.OK or abs(wrap_to_pi(result.selected_direction)) > 1e-6, (
        "the geometric gate must not select a direction of 0 rad here, despite "
        "the binary histogram (stale, via hysteresis) reporting sector 0 as free"
    )


# ---------------------------------------------------------------------------
# Numerical consistency: is_direction_safe agrees with enlargement_half_angle
# ---------------------------------------------------------------------------

def test_is_direction_safe_agrees_with_enlargement_angle():
    p = default_params()
    d = 2.0
    theta_p = 0.0
    gamma = enlargement_half_angle(d, p.r_effective)

    # Just inside the enlargement boundary: unsafe.
    assert not is_direction_safe(theta_p + 0.9 * gamma, [d], [theta_p], p.r_effective, p.d_max)
    # Just outside: safe.
    assert is_direction_safe(theta_p + 1.1 * gamma, [d], [theta_p], p.r_effective, p.d_max)
    # Exactly at the boundary: safe (lateral == r_effective, not < r_effective,
    # matching the strict "<" in is_direction_safe).
    assert is_direction_safe(theta_p + gamma, [d], [theta_p], p.r_effective, p.d_max)


# ---------------------------------------------------------------------------
# J. Wraparound obstacle
# ---------------------------------------------------------------------------

def test_j_wraparound_obstacle():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.0], angles=[deg(179)])

    # Occupied region straddles +pi/-pi; forward direction is unaffected.
    assert not result.binary_histogram[0]
    n = core.params.num_sectors
    near_180 = n // 2
    assert result.binary_histogram[near_180] or result.binary_histogram[(near_180 - 1) % n] \
        or result.binary_histogram[(near_180 + 1) % n]
    assert result.state == VFHState.OK
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


# ---------------------------------------------------------------------------
# K. Hysteresis
# ---------------------------------------------------------------------------

def test_k_hysteresis_prevents_rapid_flip():
    params = default_params()
    core = VFHPlusCore(params)

    # Call 1: d=1.2 -> magnitude 0.7, clearly above threshold_high (0.4).
    r1 = core.process(ranges=[1.2], angles=[0.0])
    assert r1.histogram[0] > params.threshold_high
    assert core.binary_state[0]

    # Call 2: same obstacle direction, further away -> magnitude 0.3, inside
    # the hysteresis band (0.2, 0.4). Must retain the previous True state.
    r2 = core.process(ranges=[2.8], angles=[0.0])
    assert params.threshold_low < r2.histogram[0] < params.threshold_high, (
        "test construction assumption: call 2's smoothed magnitude should "
        f"land inside the hysteresis band, got {r2.histogram[0]}"
    )
    assert core.binary_state[0], "sector must retain its previous True state inside the hysteresis band"

    # Call 3: obstacle gone -> magnitude 0, clearly below threshold_low.
    r3 = core.process(ranges=[params.range_max], angles=[0.0])
    assert r3.histogram[0] < params.threshold_low
    assert not core.binary_state[0], "sector must flip False once clearly below threshold_low"


# ---------------------------------------------------------------------------
# L. Previous-direction preference
# ---------------------------------------------------------------------------

def test_l_previous_direction_breaks_ties_toward_itself():
    params = default_params()
    core = VFHPlusCore(params)

    v_left = Valley(0, 0, params.sector_width_rad, candidate_direction=0.3, accepted=True)
    v_right = Valley(0, 0, params.sector_width_rad, candidate_direction=-0.3, accepted=True)

    # No history yet: fixed tie-break convention prefers left (positive).
    core.previous_direction = None
    chosen = core._select([v_left, v_right], current_heading_error=0.0)
    assert chosen.candidate_direction == pytest.approx(0.3)

    # With right previously selected, hysteresis should flip the tie to it
    # even though the fallback tie-break alone would have picked left.
    core.previous_direction = -0.3
    chosen2 = core._select([v_left, v_right], current_heading_error=0.0)
    assert chosen2.candidate_direction == pytest.approx(-0.3)


# ---------------------------------------------------------------------------
# M. No valid scan
# ---------------------------------------------------------------------------

def test_m_no_valid_scan():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[float("nan"), float("inf")], angles=[0.0, 1.0])

    assert result.state == VFHState.NO_DATA
    assert result.selected_direction is None


def test_m_empty_scan():
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[], angles=[])
    assert result.state == VFHState.NO_DATA
    assert result.selected_direction is None


# ---------------------------------------------------------------------------
# Numerical sanity checks
# ---------------------------------------------------------------------------

def test_r_robot_matches_chassis_footprint():
    r_robot = math.sqrt(0.3625 ** 2 + 0.2625 ** 2)
    assert r_robot == pytest.approx(0.4475, abs=1e-3)
    assert default_params().r_robot == pytest.approx(r_robot, abs=1e-3)


def test_enlargement_angle_known_distances():
    r_eff = default_params().r_effective  # 0.5975

    # d = 2*r_eff -> asin(0.5) = 30 deg
    assert enlargement_half_angle(2 * r_eff, r_eff) == pytest.approx(deg(30), abs=1e-6)

    # d = r_eff -> capped at 90 deg (limit of asin as ratio -> 1)
    assert enlargement_half_angle(r_eff, r_eff) == pytest.approx(deg(90), abs=1e-6)

    # d < r_eff -> also capped at 90 deg
    assert enlargement_half_angle(r_eff / 2, r_eff) == pytest.approx(deg(90), abs=1e-6)

    # d = 10*r_eff -> asin(0.1)
    assert enlargement_half_angle(10 * r_eff, r_eff) == pytest.approx(math.asin(0.1), abs=1e-6)


# ---------------------------------------------------------------------------
# Phase 1 (SUM -> MAX histogram aggregation): density-invariance tests.
# See the VFH+ Gazebo validation report and design review - real Gazebo
# LiDAR data showed a dense 286-point corridor scan driving the old summed
# magnitude to 11.55 despite ~0.78 m of actual, safe clearance. These tests
# pin the fix in isolation, at the unit level.
# ---------------------------------------------------------------------------

def test_dense_wall_does_not_scale_with_point_count():
    """N identical points at the same (d, angle) must produce the SAME
    sector magnitude as a single point there - the core density-invariance
    claim behind the SUM->MAX fix."""
    params = default_params()
    d, a = 1.5, 0.0
    hist_one = build_raw_histogram(np.array([d]), np.array([a]), params)
    hist_many = build_raw_histogram(np.array([d] * 300), np.array([a] * 300), params)
    assert np.array_equal(hist_one, hist_many)
    assert hist_one[0] == pytest.approx(point_magnitude(d, params.d_max))


def test_sparse_vs_dense_sampling_same_wall_equivalent_behavior():
    """Same straight wall (y=1.2 m, safely beyond r_effective=0.5975 m,
    parallel to the direction of travel - the real geometry of scenario J,
    which the Gazebo validation confirmed produces a clean, unblocked
    straight pass), described once by 3 points and once by ~300 points
    spanning the same length. The steering decision (state, whether
    straight-ahead is occupied, selected direction) must not depend on how
    densely the same physical surface happens to be sampled."""
    params = default_params()
    y = 1.2

    xs_sparse = [-3.0, 0.0, 3.0]
    core_sparse = VFHPlusCore(params)
    result_sparse = core_sparse.process(
        ranges=[math.hypot(x, y) for x in xs_sparse],
        angles=[math.atan2(y, x) for x in xs_sparse],
    )

    xs_dense = np.linspace(-3.0, 3.0, 300)
    core_dense = VFHPlusCore(params)
    result_dense = core_dense.process(
        ranges=[math.hypot(x, y) for x in xs_dense],
        angles=[math.atan2(y, x) for x in xs_dense],
    )

    assert result_sparse.state == VFHState.OK
    assert result_dense.state == VFHState.OK
    assert not result_sparse.binary_histogram[0]
    assert not result_dense.binary_histogram[0]
    assert result_dense.selected_direction == pytest.approx(result_sparse.selected_direction, abs=1e-3)


def test_isolated_point_magnitude_preserved_across_distances():
    """A single point's contribution to its own sector must exactly equal
    point_magnitude(d, d_max) under max-based aggregation, for every
    distance from just inside r_effective out to d_max - a direct
    regression pin confirming Phase 1 leaves the existing distance-decay
    curve itself untouched (trivially true for N=1, checked explicitly)."""
    params = default_params()
    for d in [
        params.r_effective * 0.5,
        params.r_effective,
        1.0,
        2.0,
        3.0,
        params.d_max - 0.01,
        params.d_max,
    ]:
        hist = build_raw_histogram(np.array([d]), np.array([0.0]), params)
        expected = point_magnitude(d, params.d_max)
        assert hist[0] == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# Phase 2 (one candidate/valley -> multiple sector-center candidates):
# regression + new-behavior tests, using the exact real Gazebo points from
# the VFH+ validation report where noted.
# ---------------------------------------------------------------------------

def test_wide_valley_near_goal_obstacle_not_discarded():
    """Replicates scenario B exactly: real Gazebo points for a 0.5 m cube
    ~3.75 m dead ahead (8 valid returns). The near-goal candidate (0 rad)
    is genuinely unsafe (verified directly), but the surrounding valley is
    huge and mostly safe. Under the old one-candidate-per-valley design
    this was BLOCKED with zero motion for the entire validation window.
    The valley must now yield an accepted, safe candidate instead."""
    b_deg = [-0.461, 0.461, -1.384, 1.384, -2.307, 2.307, -3.230, 3.230]
    b_d = [3.7518, 3.7518, 3.7525, 3.7525, 3.7540, 3.7540, 3.7563, 3.7563]
    angles = [deg(a) for a in b_deg]

    p = default_params()
    assert not is_direction_safe(0.0, b_d, angles, p.r_effective, p.d_max), (
        "test construction assumption: dead-ahead must be genuinely unsafe here"
    )

    core = VFHPlusCore(p)
    result = core.process(ranges=b_d, angles=angles)

    assert result.state == VFHState.OK, "the wide valley must not be discarded wholesale"
    assert len(result.rejected_valleys) == 0
    assert is_direction_safe(result.selected_direction, b_d, angles, p.r_effective, p.d_max)


def test_staggered_multi_obstacle_far_candidate_found():
    """Replicates scenario G exactly: real Gazebo points for three staggered
    0.5 m cubes. The near-goal edge of the one large valley was rejected
    (BLOCKED, zero motion for the entire validation window) under the old
    design, even though the valley's far side was safe. A safe candidate
    must now be found and selected."""
    g_deg_d = [
        (20.672, 1.8356), (19.828, 1.8399), (21.540, 1.8422), (22.431, 1.8578),
        (23.306, 1.8649), (24.183, 1.8722), (25.087, 1.8895), (25.970, 1.8972),
        (19.084, 1.8973), (26.882, 1.9156), (27.772, 1.9238), (28.693, 1.9432),
        (18.335, 1.9599), (29.619, 1.9633), (30.519, 1.9722), (31.453, 1.9931),
        (32.391, 2.0146), (17.580, 2.0282), (16.919, 2.1842), (16.148, 2.2757),
        (-6.927, 3.7873), (-6.004, 3.7939), (-7.853, 3.7962), (-8.779, 3.8059),
        (-9.706, 3.8163), (-10.637, 3.8404), (-11.567, 3.8531), (-12.497, 3.8666),
        (-13.428, 3.8807), (-5.120, 4.1788), (8.082, 5.7687), (9.034, 5.7777),
        (9.987, 5.7879), (10.941, 5.8128), (11.895, 5.8266),
    ]
    angles = [deg(a) for a, _ in g_deg_d]
    dists = [d for _, d in g_deg_d]

    core = VFHPlusCore(default_params())
    result = core.process(ranges=dists, angles=angles)

    assert result.state == VFHState.OK, "the valley must not be discarded over one failed near-goal probe"
    assert is_direction_safe(
        result.selected_direction, dists, angles, core.params.r_effective, core.params.d_max
    )


def test_candidate_fallback_picks_lowest_cost_not_farthest():
    """A single obstacle dead ahead (d=2.0 m, matching test_b) leaves both
    a modest ~20 deg deviation and a much larger ~90 deg deviation safe.
    With no previous-direction history, the existing cost function (goal +
    heading terms, both minimized by the smallest safe deviation) must
    select the modest one, not the farthest merely-safe one."""
    p = default_params()
    core = VFHPlusCore(p)
    result = core.process(ranges=[2.0], angles=[0.0])

    assert result.state == VFHState.OK
    assert not is_direction_safe(deg(15), [2.0], [0.0], p.r_effective, p.d_max), (
        "test construction assumption: 15 deg is still unsafe"
    )
    assert is_direction_safe(deg(90), [2.0], [0.0], p.r_effective, p.d_max), (
        "test construction assumption: 90 deg is also safe (a farther, higher-cost option)"
    )
    assert abs(wrap_to_pi(result.selected_direction)) < deg(45), (
        "the lowest-cost safe candidate must be chosen, not the farthest safe one"
    )


def test_wraparound_valley_multiple_candidates():
    """Same geometry as test_j_wraparound_obstacle (single point behind the
    robot, straddling +-180 deg), but explicitly confirming multiple
    candidates are now generated across the wrapped free run, and that the
    nearest-to-goal one is still selected exactly as before."""
    core = VFHPlusCore(default_params())
    result = core.process(ranges=[1.0], angles=[deg(179)])

    assert result.state == VFHState.OK
    assert len(result.candidate_valleys) > 1, "the wrapped free run should yield multiple safe candidates"
    assert abs(wrap_to_pi(result.selected_direction)) < core.params.sector_width_rad


def test_hysteresis_prefers_previous_among_multiple_safe_candidates():
    """A single obstacle dead ahead (d=2.0 m) leaves a symmetric pair of
    safe candidates (e.g. +-20 deg) with equal cost when there is no
    previous-direction history - broken by the existing fixed tie-break
    (positive preferred, per test_l). Once a previous direction on the
    NEGATIVE side is recorded, re-processing the same geometry must now
    prefer the negative side instead, confirming previous-direction
    hysteresis still functions correctly across the larger candidate set."""
    p = default_params()
    core = VFHPlusCore(p)

    result1 = core.process(ranges=[2.0], angles=[0.0])
    assert result1.state == VFHState.OK
    assert result1.selected_direction > 0.0, "no history yet: fixed tie-break prefers positive"

    core.previous_direction = deg(-20)
    result2 = core.process(ranges=[2.0], angles=[0.0])
    assert result2.state == VFHState.OK
    assert result2.selected_direction < 0.0, (
        "previous-direction hysteresis must pull selection toward the previously-chosen side"
    )


def test_obstacle_inside_r_effective_blocks_every_candidate():
    """A point strictly inside r_effective, off to the side (130 deg, not
    even dead ahead) - is_direction_safe's unconditional `d <= r_effective`
    rule makes EVERY direction unsafe regardless of the candidate tested,
    by construction. Multi-candidate sampling must not weaken this: no
    candidate anywhere may be accepted while such a point is present."""
    p = default_params()
    assert p.r_effective * 0.5 < p.r_effective  # sanity on the constant used below
    core = VFHPlusCore(p)
    result = core.process(ranges=[p.r_effective * 0.5], angles=[deg(130)])

    assert result.state == VFHState.BLOCKED
    assert len(result.candidate_valleys) == 0
    assert result.selected_direction is None


# ---------------------------------------------------------------------------
# sectors_in_span interval-overlap rewrite: regression + new-behavior tests.
# See the VFH+ K-corridor design-review addendum for the full derivation -
# these pin the exact numbers that addendum was built on.
# ---------------------------------------------------------------------------

def test_sectors_in_span_k_point_excludes_sector_0():
    """K's exact dominant point: bearing=22.09 deg, d=2.134 m ->
    half_angle=16.258 deg. True continuous danger interval is
    [5.83, 38.35] deg, which does NOT reach 0 deg. The old snap+ceil
    version nonetheless included sector 0; the new interval-overlap
    version must not."""
    result = sectors_in_span(deg(22.09), deg(16.258), 72)
    assert 0 not in result, "sector 0's range [-2.5, 2.5] does not overlap [5.83, 38.35]"
    assert 1 in result, "sector 1's range [2.5, 7.5] does overlap [5.83, 38.35]"
    # Sensible span: contiguous, bounded near the true interval's edges.
    assert sorted(result) == list(range(1, 9)), (
        f"expected sectors 1..8 (2.5..42.5 deg bracketing [5.83,38.35] deg), got {sorted(result)}"
    )


def test_sectors_in_span_general_straight_wall_never_reaches_bearing_zero():
    """General property (not just K's single point): for ANY point on a
    straight wall at fixed lateral offset y=0.9 m (> r_effective=0.5975 m)
    within the sensor's actual planning horizon (d <= d_max=4.0 m - the
    only regime `sectors_in_span` is ever actually called in, since both
    `build_raw_histogram` and `is_direction_safe` skip any point with
    d > d_max before this function would be invoked on it), the point's
    TRUE continuous danger interval can never reach bearing 0 deg.

    NOTE: beyond d_max this stops holding - e.g. x=6.95 m (d=7.01 m, well
    past d_max) gives a danger interval of [2.488, 12.27] deg, which does
    have a genuine (if tiny) mathematical overlap with sector 0's own
    range [-2.5, 2.5] deg. That is correct interval-overlap behavior, not
    a defect: `sectors_in_span` has no d_max concept of its own (it never
    did), and no caller in this module ever invokes it for a point beyond
    d_max. This test is deliberately scoped to the function's actual
    operating domain rather than asserting a property that was never true
    in general and is never exercised in practice."""
    p = default_params()
    y = 0.9
    assert y > p.r_effective, "test construction assumption: wall must be beyond r_effective"

    x_max = math.sqrt(p.d_max ** 2 - y ** 2)
    xs = [i * 0.05 for i in range(-160, 161) if i != 0 and abs(i * 0.05) <= x_max] + [0.001, -0.001]
    for x in xs:
        d = math.hypot(x, y)
        assert d <= p.d_max, "test construction assumption: stay within the sensor's planning horizon"
        bearing = math.atan2(y, x)
        half = enlargement_half_angle(d, p.r_effective)
        sectors = sectors_in_span(bearing, half, p.num_sectors)
        assert 0 not in sectors, (
            f"x={x}: point at bearing={math.degrees(bearing):.3f} deg, "
            f"half_angle={math.degrees(half):.3f} deg wrongly reaches sector 0"
        )


def test_sectors_in_span_k_replica_corridor_resolves():
    """K-replica: a representative (not just the single dominant point)
    sampling of both corridor walls at y=+-0.9 m, x in [-3.9, 3.9] m
    (within d_max), built through the full core pipeline (max-based
    histogram, existing smoothing, existing hysteresis, multi-candidate
    selection). Must NOT reproduce the all-72-sectors-occupied BLOCKED
    result observed in the real Gazebo K validation. The selected
    direction is asserted only via is_direction_safe (never hard-coded),
    per the requirement not to assume a specific angle unless
    mathematically forced."""
    p = default_params()
    xs = np.linspace(-3.9, 3.9, 79)
    ranges: list = []
    angles: list = []
    for y in (0.9, -0.9):
        for x in xs:
            ranges.append(math.hypot(x, y))
            angles.append(math.atan2(y, x))

    core = VFHPlusCore(p)
    result = core.process(ranges=ranges, angles=angles)

    assert not result.binary_histogram.all(), (
        "the corridor must not classify every sector as occupied - this is "
        "the exact failure mode observed in the real Gazebo K validation"
    )
    assert result.state == VFHState.OK
    assert len(result.candidate_valleys) > 0
    assert is_direction_safe(
        result.selected_direction, ranges, angles, p.r_effective, p.d_max
    ), "the selected direction must independently pass the exact geometric gate"


def test_sectors_in_span_exact_boundary_touch_included():
    """A point whose continuous danger interval EXACTLY touches a sector
    boundary on both sides (center=5 deg, half_angle=2.5 deg -> interval
    [2.5, 7.5], exactly matching sector 1's own range) must include the
    sectors on both sides of the touch (sector 0, touched at 2.5 deg, and
    sector 2, touched at 7.5 deg) - boundary contact counts as overlap,
    never an undershoot."""
    result = sectors_in_span(deg(5.0), deg(2.5), 72)
    assert 0 in result, "sector 0's upper edge (2.5 deg) is exactly touched by the danger interval"
    assert 1 in result
    assert 2 in result, "sector 2's lower edge (7.5 deg) is exactly touched by the danger interval"


def test_sectors_in_span_wraparound_at_pi():
    """A danger interval straddling the +-pi wraparound (center=180 deg,
    half_angle=7 deg -> interval [173, 187] deg) must return exactly the
    sectors whose true angular range overlaps that interval on either side
    of the branch cut (35: [172.5,177.5], 36: [177.5,182.5], 37's true
    center is 185 deg -> [182.5,187.5], touching at 187) - and no
    unrelated sectors (e.g. 34 or 38, which do not overlap)."""
    result = sectors_in_span(math.pi, deg(7.0), 72)
    assert sorted(result) == [35, 36, 37]
    assert 34 not in result
    assert 38 not in result


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
