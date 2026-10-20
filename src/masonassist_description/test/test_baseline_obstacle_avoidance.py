#!/usr/bin/env python3
"""Deterministic tests for scripts/validation/baseline_obstacle_avoidance.py.

Scope: the verdict logic (compute_verdict), the recording-time freshness
rule it depends on (is_fresh, applied the same way sample_row applies it),
scan_timeout parameter validation (query_live_scan_timeout's type/value
checks, exercised directly against fake GetParameters responses - no live
ROS graph), and ground-truth subprocess/parsing hardening
(_parse_gz_model_pose_output against fixture text, and query_ground_truth_pose
against a mocked subprocess.run - no live `gz` process).

No Gazebo, no running ROS graph, no wall-clock sleeps, and no external
processes are required for any test here. Tests call the driver's own real
functions with synthetic/fixture data - they do not re-derive the verdict
logic independently.

ENVIRONMENT REQUIREMENT: imports rclpy-adjacent message types used by the
driver module, so:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 -m pytest test/test_baseline_obstacle_avoidance.py -v
"""

import importlib.util
import math
import os
import sys

import pytest

_DRIVER_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "validation", "baseline_obstacle_avoidance.py")
_spec = importlib.util.spec_from_file_location("baseline_obstacle_avoidance_under_test", _DRIVER_PATH)
driver = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(driver)


# ---------------------------------------------------------------------------
# Row-building helper: mirrors exactly what sample_row() would have computed
# for a given raw (scan_age, odom_age, t_sim_s, cmd_vel, clearance, gt) input,
# using the driver's own is_fresh() - so these tests exercise the same
# freshness rule the real recording loop applies, not a re-implementation.
# ---------------------------------------------------------------------------
SCAN_TIMEOUT = 0.5


CMD_VEL_FRESH_TIMEOUT = driver.CMD_VEL_FRESH_TIMEOUT_S


def make_row(t_sim_s=0.0, scan_age=0.01, odom_age=0.01, cmd_vel_age=0.01,
             cmd_lin=0.1, cmd_ang=0.0, clearance=1.0, gt_sampled=False, gt_error=None,
             prev_t_sim_s=None, sample_delayed=False):
    return {
        "t_sim_s": t_sim_s,
        "scan_age_s": scan_age,
        "scan_fresh": driver.is_fresh(scan_age, SCAN_TIMEOUT),
        "odom_age_s": odom_age,
        "odom_fresh": driver.is_fresh(odom_age, SCAN_TIMEOUT),
        "cmd_vel_age_s": cmd_vel_age,
        "cmd_vel_fresh": driver.is_fresh(cmd_vel_age, CMD_VEL_FRESH_TIMEOUT),
        "sim_time_regression": prev_t_sim_s is not None and t_sim_s < prev_t_sim_s,
        "cmd_vel_linear_x": cmd_lin,
        "cmd_vel_angular_z": cmd_ang,
        "geom_clearance_m": clearance,
        "gt_sampled": gt_sampled,
        "gt_error": gt_error,
        "sample_delayed": sample_delayed,
    }


def make_rows(n, **kwargs):
    rows = []
    prev = None
    for i in range(n):
        row = make_row(t_sim_s=float(i), prev_t_sim_s=prev, **kwargs)
        rows.append(row)
        prev = row["t_sim_s"]
    return rows


# ---------------------------------------------------------------------------
# 1-7: compute_verdict() scenarios, via make_rows (exercises the real
# verdict function, not a re-derivation of its rules).
# ---------------------------------------------------------------------------

def test_1_all_fresh_and_valid_with_ground_truth_is_pass():
    rows = make_rows(10, gt_sampled=True)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.PASS, reasons


def test_2_scan_becomes_stale_during_recording_is_inconclusive_not_pass():
    rows = make_rows(5, gt_sampled=True)
    rows += make_rows(5, scan_age=5.0, gt_sampled=True)  # far over SCAN_TIMEOUT
    for r in rows[5:]:
        r["t_sim_s"] += 5  # keep t_sim_s monotonic across the two make_rows() calls
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("/scan was stale" in r for r in reasons)


def test_3_odometry_becomes_stale_during_recording_is_inconclusive():
    rows = make_rows(5, gt_sampled=True)
    tail = make_rows(5, odom_age=5.0, gt_sampled=True)
    for r in tail:
        r["t_sim_s"] += 5
    rows += tail
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("/odom was stale" in r for r in reasons)


def test_4_both_scan_and_odom_stale_is_inconclusive_with_both_reported():
    rows = make_rows(5, scan_age=5.0, odom_age=5.0, gt_sampled=True)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("/scan was stale" in r for r in reasons)
    assert any("/odom was stale" in r for r in reasons)


def test_5_healthy_startup_then_outage_is_inconclusive_not_concealed_by_earlier_good_rows():
    healthy = make_rows(5, gt_sampled=True)
    outage = make_rows(5, scan_age=9.0, gt_sampled=True)
    for r in outage:
        r["t_sim_s"] += 5
    rows = healthy + outage
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("/scan was stale" in r and "5/10" in r for r in reasons)


def test_6_fresh_sensors_with_safe_zero_velocity_stop_is_pass_not_failure():
    moving = make_rows(5, cmd_lin=0.1, gt_sampled=True)
    stopped = make_rows(5, cmd_lin=0.0, cmd_ang=0.0, gt_sampled=True)
    for r in stopped:
        r["t_sim_s"] += 5
    rows = moving + stopped
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.PASS, reasons
    assert any("not automatically a failure" in r or "no-safe-direction stop" in r for r in reasons)


def test_7_partial_ground_truth_failure_is_inconclusive_and_reports_errors():
    rows = make_rows(3, gt_sampled=True)
    rows += make_rows(2, gt_sampled=False, gt_error="parse failure: non-finite value parsed")
    for r in rows[3:]:
        r["t_sim_s"] += 3
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("ground truth sampling failed" in r for r in reasons)


def test_never_engaged_is_fail_even_with_perfect_sensors():
    rows = make_rows(5, cmd_lin=0.0, cmd_ang=0.0, gt_sampled=True)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.FAIL


def test_negative_clearance_is_fail_and_takes_priority_over_staleness():
    """FAIL must not be hidden by an unrelated evidence-quality problem."""
    rows = make_rows(5, clearance=-0.1, scan_age=5.0, gt_sampled=True)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.FAIL
    assert any("clearance" in r and "negative" in r for r in reasons)


def test_odom_only_clearance_without_ground_truth_caps_at_inconclusive():
    rows = make_rows(5, gt_sampled=False)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=False, interrupted=False)
    assert verdict == driver.INCONCLUSIVE


def test_13_simulation_time_backward_jump_is_inconclusive_not_pass():
    rows = make_rows(5, gt_sampled=True)
    # construct a row whose t_sim_s goes backward relative to the previous one
    bad_row = make_row(t_sim_s=1.0, prev_t_sim_s=rows[-1]["t_sim_s"], gt_sampled=True)
    rows.append(bad_row)
    assert bad_row["sim_time_regression"] is True
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("simulation time moved backward" in r for r in reasons)


def test_is_fresh_rejects_negative_age_as_not_fresh():
    """Directly exercises the fix for E: a negative age must never read as
    fresh, regardless of how small its magnitude is."""
    assert driver.is_fresh(-0.001, SCAN_TIMEOUT) is False
    assert driver.is_fresh(0.0, SCAN_TIMEOUT) is True
    assert driver.is_fresh(SCAN_TIMEOUT, SCAN_TIMEOUT) is True
    assert driver.is_fresh(SCAN_TIMEOUT + 0.001, SCAN_TIMEOUT) is False
    assert driver.is_fresh(None, SCAN_TIMEOUT) is False


# ---------------------------------------------------------------------------
# 8-9: scan_timeout parameter validation (query_live_scan_timeout), exercised
# against fake GetParameters responses - no live ROS graph needed, but the
# real method under test requires an rclpy node/client, so these construct a
# minimal real ValidationRecorder and monkeypatch only its service client's
# call, not query_live_scan_timeout itself.
# ---------------------------------------------------------------------------

class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeParamValue:
    def __init__(self, type_, double_value=0.0):
        self.type = type_
        self.double_value = double_value


class _FakeGetParametersResponse:
    def __init__(self, values):
        self.values = values


@pytest.fixture
def rclpy_node():
    import rclpy
    rclpy.init()
    node = driver.ValidationRecorder()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _patch_param_query(monkeypatch, node, response):
    """Makes query_live_scan_timeout behave as if the get_parameters service
    answered with `response`, without touching the method's own type/value
    validation logic - only the transport (wait_for_service/call_async/spin)
    is faked."""
    class _FakeClient:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            return _FakeFuture(response)

    monkeypatch.setattr(node, "create_client", lambda *a, **k: _FakeClient())
    monkeypatch.setattr(node, "destroy_client", lambda *a, **k: None)
    import rclpy
    monkeypatch.setattr(rclpy, "spin_until_future_complete", lambda *a, **k: None)


def test_8_scan_timeout_wrong_parameter_type_is_rejected(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_STRING)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is False
    assert value is None
    assert "unexpected parameter type" in reason


def test_9_scan_timeout_not_set_is_rejected(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_NOT_SET)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is False
    assert value is None
    assert "PARAMETER_NOT_SET" in reason or "unexpected parameter type" in reason


def test_9b_scan_timeout_non_finite_is_rejected(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_DOUBLE, math.nan)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is False
    assert "non-finite" in reason


def test_9c_scan_timeout_zero_is_rejected(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_DOUBLE, 0.0)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is False
    assert "zero or negative" in reason


def test_9d_scan_timeout_negative_is_rejected(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_DOUBLE, -1.0)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is False
    assert "zero or negative" in reason


def test_9e_scan_timeout_valid_double_is_accepted(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_DOUBLE, 0.5)])
    _patch_param_query(monkeypatch, rclpy_node, response)
    value, is_live, reason = rclpy_node.query_live_scan_timeout()
    assert is_live is True
    assert value == 0.5
    assert reason is None


class _FakeUnavailableClient:
    """wait_for_service() always reports the service as unavailable -
    exercises query_live_scan_timeout()'s line 348-349 branch, which no
    other test in this file reaches (every other test's fake client
    hardcodes wait_for_service() to return True)."""

    def wait_for_service(self, timeout_sec=None):
        return False

    def call_async(self, request):
        raise AssertionError(
            "call_async() must not be reached when wait_for_service() "
            "returns False - query_live_scan_timeout() should return "
            "early without ever sending a request")


def test_scan_timeout_parameter_service_unavailable_is_rejected_without_real_rclpy_node(monkeypatch):
    """Exercises query_live_scan_timeout()'s wait_for_service()->False
    branch - the one path no other test here reaches - WITHOUT creating any
    real rclpy node or context (unlike the rclpy_node-fixture tests above).

    object.__new__(driver.ValidationRecorder) builds a bare instance of the
    class while deliberately skipping Node.__init__ entirely, so rclpy.init()
    is never called and no ROS context, client, or service connection of any
    kind is created - create_client/destroy_client are then monkeypatched as
    plain instance attributes, shadowing the inherited (unused) real methods.
    This is the lightest-weight way to call the real, unmodified
    query_live_scan_timeout() method and observe its own branching logic."""
    node = object.__new__(driver.ValidationRecorder)
    monkeypatch.setattr(node, "create_client", lambda *a, **k: _FakeUnavailableClient(), raising=False)
    monkeypatch.setattr(node, "destroy_client", lambda *a, **k: None, raising=False)

    value, is_live, reason = node.query_live_scan_timeout(timeout_s=0.01)

    assert value is None
    assert is_live is False
    assert reason == f"/{driver.EXPECTED_CONTROLLER_NODE}/get_parameters service unavailable"


def test_prerun_rejects_closed_when_parameter_service_unavailable_and_fallback_disabled(monkeypatch):
    """End-to-end: run_prerun_checks() itself must not permit a PASS when
    the parameter service is unavailable and fallback is disabled - same
    no-real-rclpy-node approach as the test above."""
    node = object.__new__(driver.ValidationRecorder)
    node.latest_scan = object()
    node.latest_odom = object()
    node.latest_cmd_vel = object()
    monkeypatch.setattr(node, "create_client", lambda *a, **k: _FakeUnavailableClient(), raising=False)
    monkeypatch.setattr(node, "destroy_client", lambda *a, **k: None, raising=False)

    class Args:
        prerun_timeout = 1.0
        allow_scan_timeout_fallback = False
        scan_timeout_fallback = 0.5

    verdict, reasons, scan_timeout = driver.run_prerun_checks(node, Args())
    assert verdict == driver.INCONCLUSIVE
    assert scan_timeout is None
    assert any("service unavailable" in r for r in reasons)
    assert any("refusing to proceed without --allow-scan-timeout-fallback" in r for r in reasons)


def test_prerun_fails_closed_when_fallback_not_allowed(rclpy_node, monkeypatch):
    """End-to-end check of B's core requirement: an invalid live parameter
    must prevent the run, not silently substitute a default."""
    from rcl_interfaces.msg import ParameterType
    rclpy_node.latest_scan = object()
    rclpy_node.latest_odom = object()
    rclpy_node.latest_cmd_vel = object()
    monkeypatch.setattr(rclpy_node, "scan_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "odom_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "get_publishers_info_by_topic", lambda topic: [])
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_NOT_SET)])
    _patch_param_query(monkeypatch, rclpy_node, response)

    class Args:
        prerun_timeout = 1.0
        allow_scan_timeout_fallback = False
        scan_timeout_fallback = 0.5

    verdict, reasons, scan_timeout = driver.run_prerun_checks(rclpy_node, Args())
    assert verdict == driver.INCONCLUSIVE
    assert scan_timeout is None
    assert any("refusing to proceed without --allow-scan-timeout-fallback" in r for r in reasons)


def test_prerun_uses_fallback_only_when_explicitly_allowed(rclpy_node, monkeypatch):
    from rcl_interfaces.msg import ParameterType
    rclpy_node.latest_scan = object()
    rclpy_node.latest_odom = object()
    rclpy_node.latest_cmd_vel = object()
    monkeypatch.setattr(rclpy_node, "scan_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "odom_age_s", lambda: 0.01)

    class FakePub:
        node_name = driver.EXPECTED_CONTROLLER_NODE

    monkeypatch.setattr(rclpy_node, "get_publishers_info_by_topic", lambda topic: [FakePub()])
    response = _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_NOT_SET)])
    _patch_param_query(monkeypatch, rclpy_node, response)

    class Args:
        prerun_timeout = 1.0
        allow_scan_timeout_fallback = True
        scan_timeout_fallback = 0.5

    verdict, reasons, scan_timeout = driver.run_prerun_checks(rclpy_node, Args())
    assert verdict is None  # prerun passed, recording would proceed
    assert scan_timeout == 0.5
    assert any("FALLBACK USED" in r for r in reasons)


# ---------------------------------------------------------------------------
# 10-12: ground-truth subprocess/parsing hardening.
# ---------------------------------------------------------------------------

def test_10_malformed_ground_truth_output_is_rejected_not_fabricated():
    with pytest.raises(ValueError, match="expected at least 2 bracketed"):
        driver._parse_gz_model_pose_output("nothing useful here")


def test_11_ground_truth_output_with_nan_is_rejected():
    """The production regex's character class ([-\\d\\. e]) cannot itself
    capture a literal 'nan'/'inf' token (those letters aren't in the class),
    so text containing one simply fails the '>= 2 bracketed groups' check
    first - still correctly rejected, just via that earlier branch. This is
    documented here rather than asserting a specific message, since the
    exact rejection point is an implementation detail; what matters is that
    it is rejected, not fabricated."""
    text = "position [nan 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
    with pytest.raises(ValueError):
        driver._parse_gz_model_pose_output(text)


def test_11b_ground_truth_numeric_overflow_to_infinity_is_rejected():
    """A numeric literal that overflows float to inf (e.g. from a genuine
    scientific-notation token, unlike a literal 'nan'/'inf' string) DOES
    pass the regex and DOES reach the isfinite() check - this is the
    representative way this parser's non-finite rejection is actually
    reachable given its current character class."""
    text = "position [1e400 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
    with pytest.raises(ValueError, match="non-finite"):
        driver._parse_gz_model_pose_output(text)


def test_implausible_ground_truth_position_is_rejected():
    text = "position [999.0 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
    with pytest.raises(ValueError, match="implausible"):
        driver._parse_gz_model_pose_output(text)


def test_well_formed_ground_truth_output_is_accepted():
    text = "position [1.2 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
    x, y, yaw = driver._parse_gz_model_pose_output(text)
    assert (x, y, yaw) == (1.2, 0.5, 0.2)


def test_12_ground_truth_nonzero_exit_code_is_rejected_not_fabricated(monkeypatch):
    class FakeProc:
        returncode = 1
        stdout = "position [1.2 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
        stderr = "Error: model 'masonassist' not found"

    monkeypatch.setattr(driver.subprocess, "run", lambda *a, **k: FakeProc())
    result = driver.query_ground_truth_pose("masonassist", timeout_s=1.0)
    assert result["ok"] is False
    assert "exited 1" in result["error"]
    assert "not found" in result["error"]


def test_ground_truth_subprocess_timeout_is_reported_not_fabricated(monkeypatch):
    import subprocess as real_subprocess

    def raise_timeout(*a, **k):
        raise real_subprocess.TimeoutExpired(cmd="gz", timeout=1.0)

    monkeypatch.setattr(driver.subprocess, "run", raise_timeout)
    result = driver.query_ground_truth_pose("masonassist", timeout_s=1.0)
    assert result["ok"] is False
    assert "timed out" in result["error"]


def test_ground_truth_success_path_returns_parsed_values(monkeypatch):
    class FakeProc:
        returncode = 0
        stdout = "position [1.2 0.5 0.1]\norientation [0.0 0.0 0.2]\n"
        stderr = ""

    monkeypatch.setattr(driver.subprocess, "run", lambda *a, **k: FakeProc())
    result = driver.query_ground_truth_pose("masonassist", timeout_s=1.0)
    assert result["ok"] is True
    assert result["x"] == 1.2 and result["y"] == 0.5 and result["yaw"] == 0.2


# ---------------------------------------------------------------------------
# TASK A (compute_verdict interactions): cmd_vel freshness.
# ---------------------------------------------------------------------------

def test_cmdvel_stale_after_healthy_engagement_is_inconclusive_not_pass():
    """The critical fix: a controller that engaged then stopped republishing
    (e.g. crashed) while scan/odom stay fresh must not PASS."""
    engaged = make_rows(5, cmd_lin=0.1, cmd_vel_age=0.01, gt_sampled=True)
    frozen = make_rows(5, cmd_lin=0.1, cmd_vel_age=9.0, gt_sampled=True)  # far over CMD_VEL_FRESH_TIMEOUT
    for r in frozen:
        r["t_sim_s"] += 5
    rows = engaged + frozen
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert any("/cmd_vel was stale" in r for r in reasons)


def test_cmdvel_fresh_zero_velocity_is_not_a_failure():
    rows = make_rows(5, cmd_lin=0.1, gt_sampled=True)
    rows += make_rows(5, cmd_lin=0.0, cmd_ang=0.0, cmd_vel_age=0.01, gt_sampled=True)
    for r in rows[5:]:
        r["t_sim_s"] += 5
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.PASS, reasons


def test_cmdvel_stale_does_not_hide_independently_demonstrated_fail():
    rows = make_rows(5, clearance=-0.1, cmd_vel_age=9.0, gt_sampled=True)
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.FAIL
    assert any("negative" in r for r in reasons)


def test_compute_verdict_empty_rows_is_inconclusive():
    verdict, reasons = driver.compute_verdict([], ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert reasons


def test_no_invalid_or_missing_required_evidence_produces_pass():
    """Combined check: odom missing entirely + ground truth requested but
    failing on every row must not PASS, even though cmd_vel looks fine."""
    rows = make_rows(5, odom_age=None, gt_sampled=False, gt_error="parse failure: non-finite")
    verdict, reasons = driver.compute_verdict(rows, ground_truth_requested=True, interrupted=False)
    assert verdict == driver.INCONCLUSIVE
    assert verdict != driver.PASS


# ---------------------------------------------------------------------------
# TASK B: direct tests for sample_row() itself - not just its helpers.
# ---------------------------------------------------------------------------

class _FakeClock:
    def __init__(self, t_sim_s):
        self._t_sim_s = t_sim_s

    def now(self):
        return _FakeTime(self._t_sim_s)


class _FakeTime:
    def __init__(self, t_sim_s):
        self.nanoseconds = int(round(t_sim_s * 1e9))


class _FakeTwist:
    def __init__(self, linear_x=0.1, angular_z=0.0):
        self.linear = _Vec(linear_x)
        self.angular = _Vec(angular_z)


class _Vec:
    def __init__(self, x):
        self.x = x
        self.z = x


class _FakeSampleRowNode:
    """Lightweight stub exposing exactly the accessors sample_row() calls -
    no rclpy involved, so these tests are fast and need no ROS environment
    beyond what importing the driver module itself already requires."""

    def __init__(self, t_sim_s=0.0, scan_age=0.01, odom_age=0.01, cmd_vel_age=0.01,
                 scan_min_range=3.0, odom_xyyaw=(1.0, 0.5, 0.2), cmd_vel=None):
        self._t_sim_s = t_sim_s
        self._scan_age = scan_age
        self._odom_age = odom_age
        self._cmd_vel_age = cmd_vel_age
        self._scan_min_range = scan_min_range
        self._odom_xyyaw = odom_xyyaw
        self.latest_cmd_vel = cmd_vel if cmd_vel is not None else _FakeTwist(0.1, 0.0)

    def get_clock(self):
        return _FakeClock(self._t_sim_s)

    def scan_age_s(self):
        return self._scan_age

    def odom_age_s(self):
        return self._odom_age

    def cmd_vel_age_s(self):
        return self._cmd_vel_age

    def scan_min_range(self):
        return self._scan_min_range

    def odom_xy_yaw(self):
        return self._odom_xyyaw


class _RowArgs:
    def __init__(self, ground_truth=False, gt_timeout=1.0,
                 cmd_vel_fresh_timeout=driver.CMD_VEL_FRESH_TIMEOUT_S):
        self.ground_truth = ground_truth
        self.gt_timeout = gt_timeout
        self.cmd_vel_fresh_timeout = cmd_vel_fresh_timeout


def test_sample_row_1_all_fresh_produces_expected_fields():
    node = _FakeSampleRowNode(t_sim_s=5.0, scan_age=0.02, odom_age=0.03, cmd_vel_age=0.01,
                               scan_min_range=2.5, odom_xyyaw=(1.5, -0.2, 0.3),
                               cmd_vel=_FakeTwist(0.15, 0.05))
    row = driver.sample_row(node, _RowArgs(), scan_timeout=0.5, prev_t_sim_s=4.9)
    assert row["t_sim_s"] == 5.0
    assert row["scan_fresh"] is True
    assert row["odom_fresh"] is True
    assert row["cmd_vel_fresh"] is True
    assert row["sim_time_regression"] is False
    assert row["scan_min_range_m"] == 2.5
    assert (row["odom_x"], row["odom_y"]) == (1.5, -0.2)
    assert row["cmd_vel_linear_x"] == 0.15 and row["cmd_vel_angular_z"] == 0.05


def test_sample_row_2_stale_scan_is_represented_correctly():
    node = _FakeSampleRowNode(scan_age=9.0)
    row = driver.sample_row(node, _RowArgs(), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["scan_fresh"] is False
    assert row["scan_age_s"] == 9.0
    # unrelated fields are unaffected - staleness isn't all-or-nothing
    assert row["odom_fresh"] is True


def test_sample_row_3_stale_odometry_is_represented_correctly():
    node = _FakeSampleRowNode(odom_age=9.0)
    row = driver.sample_row(node, _RowArgs(), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["odom_fresh"] is False
    assert row["scan_fresh"] is True


def test_sample_row_4_stale_command_is_represented_correctly():
    node = _FakeSampleRowNode(cmd_vel_age=9.0)
    row = driver.sample_row(node, _RowArgs(cmd_vel_fresh_timeout=0.3), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["cmd_vel_fresh"] is False
    assert row["scan_fresh"] is True and row["odom_fresh"] is True


def test_sample_row_5_ground_truth_failure_recorded_not_fabricated(monkeypatch):
    node = _FakeSampleRowNode()
    monkeypatch.setattr(driver, "query_ground_truth_pose",
                         lambda model, timeout_s: {"ok": False, "error": "parse failure: boom",
                                                    "latency": 0.01})
    row = driver.sample_row(node, _RowArgs(ground_truth=True), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["gt_sampled"] is False
    assert row["gt_error"] == "parse failure: boom"
    assert row["gt_x"] is None and row["gt_y"] is None


def test_sample_row_6_missing_or_invalid_sensor_data_does_not_become_fresh():
    node = _FakeSampleRowNode(scan_age=None, odom_age=-0.5, cmd_vel_age=None)
    row = driver.sample_row(node, _RowArgs(), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["scan_fresh"] is False
    assert row["odom_fresh"] is False
    assert row["cmd_vel_fresh"] is False


def test_sample_row_7a_clearance_source_is_ground_truth_when_gt_succeeds(monkeypatch):
    node = _FakeSampleRowNode(odom_xyyaw=(1.0, 0.0, 0.0))
    monkeypatch.setattr(driver, "query_ground_truth_pose",
                         lambda model, timeout_s: {"ok": True, "x": 1.9, "y": 0.1, "yaw": 0.0,
                                                    "latency": 0.01})
    row = driver.sample_row(node, _RowArgs(ground_truth=True), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["clearance_source"] == "ground_truth"
    assert row["geom_clearance_m"] == pytest.approx(driver.box_clearance(
        1.9, 0.1, driver.OBSTACLE_CENTER_XY[0], driver.OBSTACLE_CENTER_XY[1],
        driver.OBSTACLE_HALF_EXTENT, driver.R_ROBOT))


def test_sample_row_7b_clearance_source_is_odom_when_ground_truth_not_requested():
    node = _FakeSampleRowNode(odom_xyyaw=(1.0, 0.0, 0.0))
    row = driver.sample_row(node, _RowArgs(ground_truth=False), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["clearance_source"] == "odom (not independently verified)"
    assert row["geom_clearance_m"] is not None


def test_sample_row_7c_clearance_unavailable_when_gt_mode_selected_and_query_fails(monkeypatch):
    """TASK D: in ground-truth mode, a failed query must leave clearance
    unavailable for that row - never silently substitute odometry."""
    node = _FakeSampleRowNode(odom_xyyaw=(1.0, 0.0, 0.0))
    monkeypatch.setattr(driver, "query_ground_truth_pose",
                         lambda model, timeout_s: {"ok": False, "error": "timed out", "latency": 2.0})
    row = driver.sample_row(node, _RowArgs(ground_truth=True), scan_timeout=0.5, prev_t_sim_s=None)
    assert row["gt_sampled"] is False
    assert row["geom_clearance_m"] is None
    assert row["clearance_source"] is None


# ---------------------------------------------------------------------------
# TASK C: run_prerun_checks() controller-identity branches.
# ---------------------------------------------------------------------------

def _valid_scan_timeout_response():
    from rcl_interfaces.msg import ParameterType
    return _FakeGetParametersResponse([_FakeParamValue(ParameterType.PARAMETER_DOUBLE, 0.5)])


def _prerun_args():
    class Args:
        prerun_timeout = 1.0
        allow_scan_timeout_fallback = False
        scan_timeout_fallback = 0.5
    return Args()


def test_prerun_rejects_when_excluded_controller_present(rclpy_node, monkeypatch):
    rclpy_node.latest_scan = object()
    rclpy_node.latest_odom = object()
    rclpy_node.latest_cmd_vel = object()
    monkeypatch.setattr(rclpy_node, "scan_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "odom_age_s", lambda: 0.01)

    class FakePubExpected:
        node_name = driver.EXPECTED_CONTROLLER_NODE

    class FakePubExcluded:
        node_name = driver.EXCLUDED_CONTROLLER_NODE

    monkeypatch.setattr(rclpy_node, "get_publishers_info_by_topic",
                         lambda topic: [FakePubExpected(), FakePubExcluded()])
    _patch_param_query(monkeypatch, rclpy_node, _valid_scan_timeout_response())

    verdict, reasons, scan_timeout = driver.run_prerun_checks(rclpy_node, _prerun_args())
    assert verdict == driver.INCONCLUSIVE
    assert any("excluded controller" in r for r in reasons)


def test_prerun_rejects_when_expected_controller_absent(rclpy_node, monkeypatch):
    rclpy_node.latest_scan = object()
    rclpy_node.latest_odom = object()
    rclpy_node.latest_cmd_vel = object()
    monkeypatch.setattr(rclpy_node, "scan_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "odom_age_s", lambda: 0.01)

    class FakePubOther:
        node_name = "some_other_node"

    monkeypatch.setattr(rclpy_node, "get_publishers_info_by_topic", lambda topic: [FakePubOther()])
    _patch_param_query(monkeypatch, rclpy_node, _valid_scan_timeout_response())

    verdict, reasons, scan_timeout = driver.run_prerun_checks(rclpy_node, _prerun_args())
    assert verdict == driver.INCONCLUSIVE
    assert any("expected controller" in r for r in reasons)


def test_prerun_passes_when_expected_present_and_excluded_absent(rclpy_node, monkeypatch):
    rclpy_node.latest_scan = object()
    rclpy_node.latest_odom = object()
    rclpy_node.latest_cmd_vel = object()
    monkeypatch.setattr(rclpy_node, "scan_age_s", lambda: 0.01)
    monkeypatch.setattr(rclpy_node, "odom_age_s", lambda: 0.01)

    class FakePubExpected:
        node_name = driver.EXPECTED_CONTROLLER_NODE

    monkeypatch.setattr(rclpy_node, "get_publishers_info_by_topic", lambda topic: [FakePubExpected()])
    _patch_param_query(monkeypatch, rclpy_node, _valid_scan_timeout_response())

    verdict, reasons, scan_timeout = driver.run_prerun_checks(rclpy_node, _prerun_args())
    assert verdict is None
    assert scan_timeout == 0.5


# ---------------------------------------------------------------------------
# TASK E: CLI help explicitly states ground truth is required for PASS.
# ---------------------------------------------------------------------------

def test_cli_help_states_ground_truth_required_for_pass(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["baseline_obstacle_avoidance.py", "--help"])
    with pytest.raises(SystemExit):
        driver.parse_args()
    out = capsys.readouterr().out
    assert "REQUIRED FOR A PASS" in out
