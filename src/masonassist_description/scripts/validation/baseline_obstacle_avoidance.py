#!/usr/bin/env python3
"""Passive validation driver for the baseline Gazebo obstacle-avoidance scenario.

SCENARIO: worlds/masonassist.world's existing `lidar_test_obstacle` (a static
0.5 x 0.5 x 0.5 m box at world pose (2.0, 0, 0.25)), robot at its default
spawn (x=0, y=0, z=0.1, yaw=0), VFH+ enabled, simple obstacle avoidance
disabled. This is a controlled validation experiment, not a claim of
physical-robot safety.

WHAT THIS SCRIPT DOES:
- Subscribes to /scan, /odom, /cmd_vel. Never publishes anything, to any
  topic, under any condition.
- Runs pre-run prerequisite checks (topics present, scan/odom fresh against
  vfh_plus_node's own live scan_timeout parameter, vfh_plus_node is actually
  the node publishing /cmd_vel, simple_obstacle_avoidance is not) before
  recording anything that would count toward a PASS.
- Records a timestamped CSV for a fixed duration.
- Optionally polls Gazebo ground-truth pose via `gz model -p` (subprocess,
  --ground-truth flag) - kept clearly distinct from /odom in the output.
- Computes a geometric clearance estimate against the obstacle's known
  world-frame pose and reports PASS / FAIL / INCONCLUSIVE with written
  rationale. Never silently treats missing data as success.

WHAT THIS SCRIPT DOES NOT DO:
- Does not launch, stop, restart, or configure Gazebo or any ROS node.
- Does not publish /cmd_vel or anything else.
- Does not implement recovery/retry/reverse maneuvers.
- Does not modify production code, world files, or launch files.

REQUIRED LAUNCH COMMAND (run first, separately, in another terminal):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    ros2 launch masonassist_description gazebo.launch.py \\
        enable_vfh_plus:=true enable_obstacle_avoidance:=false

THEN RUN THIS SCRIPT (separately, after the above is up and settled):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/validation/baseline_obstacle_avoidance.py \\
        [--ground-truth] [--duration 30] [--output PATH]

ENDING A RUN SAFELY: Ctrl+C this script at any time - it catches the
interrupt, writes out whatever rows it already recorded plus a summary
noting the early stop, and exits. This script never starts Gazebo or any
background process, so there is nothing of its own left to clean up.
Separately Ctrl+C the `ros2 launch` terminal when you are done with the
simulation; this script does not do that for you and never touches it.

TIME BASIS: CSV rows use this node's ROS clock (use_sim_time=True is forced
on this node, matching every other node in this launch), so t_sim_s is
simulation time, not wall time. /scan and /odom ages are computed from each
message's own header.stamp (sim time) against that same clock - directly
comparable. geometry_msgs/Twist has no header, so /cmd_vel "age" is instead
time-since-this-subscriber-last-received-a-message on this node's own clock
- a liveness proxy, not a true publish-time age. If --ground-truth is used,
each `gz model -p` sample is a *blocking subprocess call* made once per loop
tick; its own wall-clock round-trip latency is recorded in
gt_sample_wall_latency_s so a reader can judge how loosely it is aligned to
the same row's ROS-sampled fields - it is not a hard real-time guarantee.

COLLISION CLEARANCE CAVEAT (do not over-read this number): R_ROBOT below is
VFHPlusCore's own r_robot (0.4475 m), the circumscribed-circle radius of the
robot's real 0.725 x 0.525 m rectangular chassis footprint (see
vfh_plus_core.py's VFHParams docstring). A circle of this radius fully
contains the real footprint, so a POSITIVE clearance against it is a valid
guarantee that the real mesh cannot be touching the obstacle. A NEGATIVE
clearance only means this conservative circular bound was crossed - it does
NOT by itself prove the real rectangular mesh made contact, since the real
edge is closer than r_robot only along the exact diagonal direction. Also
note /scan's "minimum valid range" is measured from the LiDAR's own mount
point (0.30, 0, 0.19 relative to base_link, per model.sdf), not from
base_link's origin - the two are related but not numerically interchangeable,
and this script does not attempt that reconciliation. GT_PLAUSIBLE_BOUND_M
below is a scenario-specific sanity bound (this room/obstacle layout is tiny),
not a physical constant and not proof that an accepted pose is correct.

VERDICT PRECEDENCE (checked in this order, every time):
1. FAIL - trustworthy evidence demonstrates a defined failure condition
   (controller never engaged /cmd_vel, or geometric clearance went negative).
   A demonstrated failure is never hidden by an unrelated outage.
2. INCONCLUSIVE - required evidence was missing, stale, invalid, or affected
   by a simulation-time anomaly during recording (scan/odom staleness after
   a healthy prerun, a sim-time backward jump, incomplete/invalid required
   ground-truth sampling, or odom-only clearance). None of these is itself a
   demonstrated behavioral failure - a sensor outage or invalid sample makes
   the run's evidence untrustworthy, not the robot's behavior wrong.
3. PASS - only if neither of the above triggered: every required pass
   criterion was met on trustworthy evidence for the whole recorded run.
A zero-velocity /cmd_vel command with fresh sensor data is not automatically
a failure (it may be a correct safe stop) - it is reported as a fact for a
human to interpret, never auto-classified either way. This is distinct
from /cmd_vel NEVER going non-zero across the entire recorded run, which
IS classified FAIL (see "controller never engaged" above).

SCENARIO-SPECIFIC CAVEAT on that "never engaged" FAIL rule: it is specific
to this baseline scenario, where a static obstacle sits directly ahead of
the robot's default spawn pose, so some initial motion is the expected
correct behavior. Do not generalize this rule to a different scenario
where remaining stationary for the whole run could itself be correct (e.g.
a scenario with no reachable obstacle, or one that starts the robot
already in a state where stopping is the right call) - in such a scenario,
treating "never went non-zero" as a FAIL would be wrong, and this check
would need to be reconsidered, not reused as-is.

SCAN_TIMEOUT FALLBACK: by default (--allow-scan-timeout-fallback is OFF),
an unavailable, unset, wrongly-typed, non-finite, zero, or negative live
scan_timeout parameter on vfh_plus_node causes an explicit INCONCLUSIVE
preflight failure - the run does not proceed, and no timeout is invented.
Passing --allow-scan-timeout-fallback opts into substituting
--scan-timeout-fallback instead, and that substitution is always recorded
loudly in the summary (that it was used, its value, and why the live value
could not be used) - it is never silent.

CMD_VEL FRESHNESS: vfh_plus_node.py republishes /cmd_vel from one of two
places - its scan_callback (driven by /scan arrival, ~10 Hz at the LiDAR's
configured update_rate) or its own watchdog_callback, which runs on a fixed
0.1 s timer regardless of scan/odom health and publishes a zero Twist once
staleness is detected. A controller process that is actually still alive
therefore republishes /cmd_vel at least once per ~0.1 s under every
condition - healthy, stale-upstream, or stopped. CMD_VEL_FRESH_TIMEOUT_S
below (0.3 s = 3x that period, generous enough to absorb this validator's
own sampling jitter) is deliberately NOT the same as scan_timeout (0.5 s):
reusing scan_timeout would tolerate a dead controller for up to 5x longer
than its own expected republish interval before detecting it. A row whose
/cmd_vel has not been refreshed within this window is treated as invalid
evidence (the controller may have crashed while /scan or /odom, driven by
other still-alive nodes, remain fresh) and caps the run at INCONCLUSIVE -
it is never confused with a demonstrated obstacle-avoidance FAIL, and a
fresh zero-velocity command (still actively being republished, just with
zero value) is never treated as stale or as a failure merely because its
value is zero.

ASSUMPTION DISCLOSURE: the 0.3 s figure above is a conservative threshold
derived from reading vfh_plus_node.py's source (its timer period and
scan-driven publish path) - it has NOT been measured against that
controller's actual live publish cadence under real DDS/executor
scheduling jitter. If a live run shows this threshold is too tight (false
staleness under normal jitter) or too loose, it should be revisited against
real measurements, not assumed correct from source-reading alone.

GROUND-TRUTH EVIDENCE MODE: when --ground-truth is passed, this run is in
ground-truth validation mode, and a failed ground-truth query on a given
row leaves that row's geom_clearance_m/clearance_source as None rather
than silently substituting odometry - odometry-derived clearance is only
ever used when --ground-truth was not requested at all (odometry-only
mode). Ground truth is required for a PASS in this baseline validation:
without --ground-truth, or with any row's ground-truth sampling
incomplete, the run is capped at INCONCLUSIVE regardless of how clean
everything else looks.

SIMULATION-TIME REGRESSION DETECTION LIMITATION: sim_time_regression only
compares each recorded row's t_sim_s against the immediately preceding
recorded row's - it has no visibility between samples. A backward jump
followed by a full recovery that both happen strictly between two
recorded samples (faster than this script's own sampling rate) would not
be detected. Widening this would require either a higher sampling rate or
a separate out-of-band clock monitor, neither of which this hardening
pass implements.
"""

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
from vfh_plus_core import VFHParams  # noqa: E402

PASS, FAIL, INCONCLUSIVE = "PASS", "FAIL", "INCONCLUSIVE"
EXIT_CODES = {PASS: 0, FAIL: 1, INCONCLUSIVE: 2}

# Known obstacle geometry, transcribed directly from worlds/masonassist.world's
# <model name="lidar_test_obstacle"> block (static, axis-aligned, world frame).
# Not an invented threshold - the world file is frozen for this task, so this
# is a direct read of its existing content, not a new value.
OBSTACLE_CENTER_XY = (2.0, 0.0)
OBSTACLE_HALF_EXTENT = 0.25  # half of the 0.5 x 0.5 x 0.5 m box

# Imported, not hardcoded, so this script cannot drift out of sync with the
# live VFH+ core if its footprint parameters ever change.
R_ROBOT = VFHParams().r_robot

EXPECTED_CONTROLLER_NODE = "vfh_plus_node"
EXCLUDED_CONTROLLER_NODE = "simple_obstacle_avoidance"
SPAWN_MODEL_NAME = "masonassist"  # must match launch/gazebo.launch.py's ros_gz_sim create -name

FALLBACK_SCAN_TIMEOUT_S = 0.5  # only used if --allow-scan-timeout-fallback is explicitly passed

# See the module docstring's CMD_VEL FRESHNESS section: 3x vfh_plus_node's own
# 0.1s watchdog/scan-driven republish period, deliberately not scan_timeout.
# This is a conservative, SOURCE-DERIVED threshold (from reading the
# controller's timer/publish paths) - it has not been measured against that
# controller's actual live publish cadence under real DDS/executor
# scheduling jitter. Revisit against live measurements before trusting it
# as tuned, not just plausible.
CMD_VEL_FRESH_TIMEOUT_S = 0.3

# Scenario-specific sanity bound on parsed ground-truth coordinates (this
# room/obstacle layout is a few meters across) - not a physical constant,
# and accepting a value under this bound is not proof the pose is correct.
GT_PLAUSIBLE_BOUND_M = 50.0

CSV_FIELDS = [
    "t_sim_s", "t_wall_s", "tick_duration_s", "sample_delayed",
    "scan_age_s", "scan_min_range_m", "scan_fresh",
    "odom_age_s", "odom_x", "odom_y", "odom_yaw_deg", "odom_fresh",
    "sim_time_regression",
    "cmd_vel_age_s", "cmd_vel_fresh", "cmd_vel_linear_x", "cmd_vel_angular_z",
    "gt_sampled", "gt_x", "gt_y", "gt_yaw_deg", "gt_sample_wall_latency_s", "gt_error",
    "geom_clearance_m", "clearance_source",
]

LAUNCH_REMINDER = (
    "Assumes this was already launched separately, and is still running:\n"
    "  source /opt/ros/jazzy/setup.bash && source install/setup.bash\n"
    "  ros2 launch masonassist_description gazebo.launch.py "
    "enable_vfh_plus:=true enable_obstacle_avoidance:=false\n"
    "This script does not launch, stop, or configure Gazebo or any ROS node.\n"
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def is_fresh(age_s, timeout_s):
    """A negative age (simulation-time reset/backward jump) is never fresh -
    it is invalid, not 'fresher than fresh'."""
    return age_s is not None and 0.0 <= age_s <= timeout_s


def box_clearance(px, py, cx, cy, half_extent, r_robot):
    """Planar distance from a circle of radius r_robot centered at (px, py)
    to the nearest surface of an axis-aligned square, minus r_robot. See the
    module docstring's COLLISION CLEARANCE CAVEAT before interpreting a
    negative result as a proven collision."""
    dx = max(abs(px - cx) - half_extent, 0.0)
    dy = max(abs(py - cy) - half_extent, 0.0)
    return math.hypot(dx, dy) - r_robot


def _parse_gz_model_pose_output(stdout_text):
    """Pure parsing function - no subprocess involved, so it is directly
    unit-testable against representative fixture text. Returns (x, y,
    yaw_rad) or raises ValueError with a specific reason.

    UNVERIFIED ASSUMPTION, not re-confirmed against a live Gazebo process in
    this hardening pass: that the second-to-last and last bracketed numeric
    groups in `gz model -p`'s text output are position and orientation(rpy)
    respectively. This mirrors this project's earlier SLAM-mapping scratchpad
    convention but live compatibility is not claimed here."""
    nums = [list(map(float, m.split())) for m in re.findall(r"\[([-\d\. e]+)\]", stdout_text)]
    if len(nums) < 2:
        raise ValueError(f"expected at least 2 bracketed numeric groups, found {len(nums)}")
    pos, ori = nums[-2], nums[-1]
    if len(pos) < 2 or len(ori) < 3:
        raise ValueError("position or orientation group has too few components")
    x, y, yaw = pos[0], pos[1], ori[2]
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError(f"non-finite value parsed: x={x} y={y} yaw={yaw}")
    if abs(x) > GT_PLAUSIBLE_BOUND_M or abs(y) > GT_PLAUSIBLE_BOUND_M:
        raise ValueError(f"implausible position for this scenario: x={x} y={y} "
                          f"(sanity bound={GT_PLAUSIBLE_BOUND_M} m, not a physical limit)")
    return x, y, yaw


def query_ground_truth_pose(model_name, timeout_s):
    """Blocking `gz model -p` subprocess call. Returns a dict:
    {"ok": True, "x", "y", "yaw", "latency"} on success, or
    {"ok": False, "error": <reason>, "latency"} on any failure - a failed or
    invalid sample is always reported explicitly, never silently dropped or
    replaced with fabricated coordinates."""
    t0 = time.monotonic()
    try:
        proc = subprocess.run(
            ["gz", "model", "-m", model_name, "-p"],
            capture_output=True, text=True, timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"gz model query timed out after {timeout_s}s",
                "latency": time.monotonic() - t0}
    except (FileNotFoundError, OSError) as e:
        return {"ok": False, "error": f"gz model query failed to start: {e}",
                "latency": time.monotonic() - t0}
    latency = time.monotonic() - t0
    if proc.returncode != 0:
        return {"ok": False, "latency": latency,
                "error": f"gz model exited {proc.returncode}, stderr: {proc.stderr.strip()[:200]!r}"}
    try:
        x, y, yaw = _parse_gz_model_pose_output(proc.stdout)
    except ValueError as e:
        return {"ok": False, "error": f"parse failure: {e}", "latency": latency}
    return {"ok": True, "x": x, "y": y, "yaw": yaw, "latency": latency}


class ValidationRecorder(Node):
    """Subscribes only. Never creates a publisher - verified by inspection:
    there is no create_publisher call anywhere in this class or module."""

    def __init__(self):
        super().__init__(
            "baseline_obstacle_avoidance_validator",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.latest_scan = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.last_cmd_vel_recv_time = None  # Twist has no header; this is our own recv clock

        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _cmd_vel_cb(self, msg):
        self.latest_cmd_vel = msg
        self.last_cmd_vel_recv_time = self.get_clock().now()

    def scan_age_s(self):
        if self.latest_scan is None:
            return None
        return (self.get_clock().now() - Time.from_msg(self.latest_scan.header.stamp)).nanoseconds / 1e9

    def odom_age_s(self):
        if self.latest_odom is None:
            return None
        return (self.get_clock().now() - Time.from_msg(self.latest_odom.header.stamp)).nanoseconds / 1e9

    def cmd_vel_age_s(self):
        if self.last_cmd_vel_recv_time is None:
            return None
        return (self.get_clock().now() - self.last_cmd_vel_recv_time).nanoseconds / 1e9

    def scan_min_range(self):
        if self.latest_scan is None:
            return None
        msg = self.latest_scan
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(valid) if valid else None

    def odom_xy_yaw(self):
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose
        return p.position.x, p.position.y, yaw_from_quaternion(p.orientation)

    def query_live_scan_timeout(self, timeout_s=3.0):
        """Fetch vfh_plus_node's actual scan_timeout parameter via its own
        get_parameters service, checking both type and value - never
        fabricates or assumes a timeout. Returns (value_or_None, is_live,
        reason_or_None). value is only non-None when is_live is True; the
        caller decides whether an explicit fallback is permitted."""
        client = self.create_client(GetParameters, f"/{EXPECTED_CONTROLLER_NODE}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                return None, False, f"/{EXPECTED_CONTROLLER_NODE}/get_parameters service unavailable"
            future = client.call_async(GetParameters.Request(names=["scan_timeout"]))
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            result = future.result() if future.done() else None
            if result is None:
                return None, False, "get_parameters call did not complete"
            if not result.values:
                return None, False, "get_parameters response had no values"
            pv = result.values[0]
            if pv.type != ParameterType.PARAMETER_DOUBLE:
                return None, False, (f"scan_timeout has unexpected parameter type {pv.type} "
                                      f"(expected PARAMETER_DOUBLE={ParameterType.PARAMETER_DOUBLE}; "
                                      f"0=PARAMETER_NOT_SET means the parameter does not exist)")
            value = pv.double_value
            if not math.isfinite(value):
                return None, False, f"scan_timeout is non-finite ({value})"
            if value <= 0:
                return None, False, f"scan_timeout is zero or negative ({value})"
            return value, True, None
        finally:
            self.destroy_client(client)


def run_prerun_checks(node, args):
    """Returns (verdict_or_None, notes, scan_timeout). verdict is
    INCONCLUSIVE if any prerequisite failed, else None (continue to
    recording) with informational notes about what was confirmed."""
    deadline = time.monotonic() + args.prerun_timeout
    while time.monotonic() < deadline and not (
        node.latest_scan and node.latest_odom and node.latest_cmd_vel is not None
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    reasons = []
    if node.latest_scan is None:
        reasons.append("no /scan message received within prerun_timeout")
    if node.latest_odom is None:
        reasons.append("no /odom message received within prerun_timeout")
    if node.latest_cmd_vel is None:
        reasons.append("no /cmd_vel message received within prerun_timeout")
    if reasons:
        return INCONCLUSIVE, reasons, None

    scan_timeout, is_live, live_fail_reason = node.query_live_scan_timeout()
    if is_live:
        reasons.append(f"scan_timeout={scan_timeout}s (queried live from {EXPECTED_CONTROLLER_NODE})")
    elif args.allow_scan_timeout_fallback:
        scan_timeout = args.scan_timeout_fallback
        reasons.append(
            f"FALLBACK USED for scan_timeout={scan_timeout}s - live value could not be used "
            f"({live_fail_reason}) - --allow-scan-timeout-fallback was explicitly passed"
        )
    else:
        reasons.append(f"scan_timeout could not be validated live: {live_fail_reason}")
        reasons.append("refusing to proceed without --allow-scan-timeout-fallback "
                        "(disabled by default - no timeout is invented)")
        return INCONCLUSIVE, reasons, None

    scan_age, odom_age = node.scan_age_s(), node.odom_age_s()
    if not is_fresh(scan_age, scan_timeout):
        reasons.append(f"/scan stale, invalid, or unavailable (age={scan_age})")
    if not is_fresh(odom_age, scan_timeout):
        reasons.append(f"/odom stale, invalid, or unavailable (age={odom_age})")

    publishers = node.get_publishers_info_by_topic("/cmd_vel")
    pub_names = sorted({p.node_name for p in publishers})
    reasons.append(f"/cmd_vel publishers observed: {pub_names}")
    if EXCLUDED_CONTROLLER_NODE in pub_names:
        reasons.append(f"excluded controller '{EXCLUDED_CONTROLLER_NODE}' is publishing /cmd_vel")
    if EXPECTED_CONTROLLER_NODE not in pub_names:
        reasons.append(f"expected controller '{EXPECTED_CONTROLLER_NODE}' not found among /cmd_vel publishers")

    if any(r.startswith(("/scan stale", "/odom stale", "excluded controller", "expected controller"))
           for r in reasons):
        return INCONCLUSIVE, reasons, scan_timeout
    return None, reasons, scan_timeout


def sample_row(node, args, scan_timeout, prev_t_sim_s):
    """prev_t_sim_s: the previous row's t_sim_s (or None for the first row),
    used only to detect a simulation-time backward jump between samples."""
    now = node.get_clock().now()
    row = dict.fromkeys(CSV_FIELDS)
    row["t_sim_s"] = now.nanoseconds / 1e9
    row["t_wall_s"] = time.time()
    row["sim_time_regression"] = prev_t_sim_s is not None and row["t_sim_s"] < prev_t_sim_s
    row["scan_age_s"] = node.scan_age_s()
    row["scan_min_range_m"] = node.scan_min_range()
    row["scan_fresh"] = is_fresh(row["scan_age_s"], scan_timeout)
    row["odom_age_s"] = node.odom_age_s()
    row["odom_fresh"] = is_fresh(row["odom_age_s"], scan_timeout)
    row["cmd_vel_age_s"] = node.cmd_vel_age_s()
    row["cmd_vel_fresh"] = is_fresh(row["cmd_vel_age_s"], args.cmd_vel_fresh_timeout)
    row["gt_sampled"] = False
    row["gt_error"] = None

    odom = node.odom_xy_yaw()
    if odom:
        row["odom_x"], row["odom_y"], yaw = odom
        row["odom_yaw_deg"] = math.degrees(yaw)
    if node.latest_cmd_vel is not None:
        row["cmd_vel_linear_x"] = node.latest_cmd_vel.linear.x
        row["cmd_vel_angular_z"] = node.latest_cmd_vel.angular.z

    # GROUND-TRUTH EVIDENCE MODE (see module docstring): --ground-truth
    # selects ground-truth validation mode for the whole run. A failed query
    # in that mode leaves this row's clearance unavailable - it never falls
    # back to odometry, so the source of every reported clearance is always
    # exactly what was actually used, never a silent substitution.
    clearance_xy, clearance_source = None, None
    if args.ground_truth:
        gt = query_ground_truth_pose(SPAWN_MODEL_NAME, args.gt_timeout)
        row["gt_sample_wall_latency_s"] = gt.get("latency")
        if gt["ok"]:
            row["gt_sampled"] = True
            row["gt_x"], row["gt_y"], row["gt_yaw_deg"] = gt["x"], gt["y"], math.degrees(gt["yaw"])
            clearance_xy, clearance_source = (gt["x"], gt["y"]), "ground_truth"
        else:
            row["gt_error"] = gt["error"]
    elif odom:
        clearance_xy, clearance_source = (row["odom_x"], row["odom_y"]), "odom (not independently verified)"

    if clearance_xy:
        row["geom_clearance_m"] = box_clearance(
            clearance_xy[0], clearance_xy[1],
            OBSTACLE_CENTER_XY[0], OBSTACLE_CENTER_XY[1],
            OBSTACLE_HALF_EXTENT, R_ROBOT,
        )
        row["clearance_source"] = clearance_source
    return row


def compute_verdict(rows, ground_truth_requested, interrupted):
    """Implements the VERDICT PRECEDENCE documented in the module docstring:
    FAIL (demonstrated failure) takes priority over INCONCLUSIVE (evidence
    quality problem), which takes priority over PASS. Freshness/regression
    flags are read from what sample_row() already computed per-row (using
    the single scan_timeout resolved once at prerun), not recomputed here."""
    if not rows:
        return INCONCLUSIVE, ["no rows recorded during the run window"]

    reasons = []
    if interrupted:
        reasons.append("run was interrupted (Ctrl+C) before its configured duration elapsed")

    cmd_vel_ever_nonzero = any(
        abs(r["cmd_vel_linear_x"] or 0.0) > 1e-3 or abs(r["cmd_vel_angular_z"] or 0.0) > 1e-3
        for r in rows
    )
    clearances = [r["geom_clearance_m"] for r in rows if r["geom_clearance_m"] is not None]
    min_clearance = min(clearances) if clearances else None

    # 1. FAIL - a demonstrated failure is never hidden by an unrelated outage.
    # "Never engaged" -> FAIL is a SCENARIO-SPECIFIC rule for this baseline
    # obstacle-ahead scenario (see the module docstring's SCENARIO-SPECIFIC
    # CAVEAT) - do not reuse this check unmodified for a scenario where
    # remaining stationary for the whole run could itself be correct.
    if not cmd_vel_ever_nonzero:
        reasons.append("cmd_vel never went non-zero during the run - controller did not engage")
        return FAIL, reasons
    if min_clearance is not None and min_clearance < 0:
        reasons.append(
            f"geometric clearance against r_robot={R_ROBOT:.4f} m went negative "
            f"(min={min_clearance:.3f} m) - conservative collision boundary violated. "
            f"Does not by itself prove the real chassis mesh made contact (see module "
            f"docstring's COLLISION CLEARANCE CAVEAT), but this is not a PASS."
        )
        return FAIL, reasons

    # 2. INCONCLUSIVE - evidence-quality problems only; none of these is
    # itself a demonstrated behavioral failure.
    inconclusive = []
    stale_scan = [r for r in rows if not r["scan_fresh"]]
    stale_odom = [r for r in rows if not r["odom_fresh"]]
    stale_cmd_vel = [r for r in rows if not r["cmd_vel_fresh"]]
    regressed = [r for r in rows if r["sim_time_regression"]]
    if stale_scan:
        inconclusive.append(
            f"/scan was stale, invalid, or unavailable on {len(stale_scan)}/{len(rows)} recorded "
            f"rows (first at t_sim_s={stale_scan[0]['t_sim_s']:.2f}) - outage during recording "
            f"after a healthy prerun; not concealed by earlier good samples"
        )
    if stale_odom:
        inconclusive.append(
            f"/odom was stale, invalid, or unavailable on {len(stale_odom)}/{len(rows)} recorded "
            f"rows (first at t_sim_s={stale_odom[0]['t_sim_s']:.2f})"
        )
    if stale_cmd_vel:
        inconclusive.append(
            f"/cmd_vel was stale, invalid, or unavailable on {len(stale_cmd_vel)}/{len(rows)} "
            f"recorded rows (first at t_sim_s={stale_cmd_vel[0]['t_sim_s']:.2f}) - the controller "
            f"may have stopped republishing (e.g. crashed) while other sensors stayed fresh; this "
            f"is an evidence outage, not itself a demonstrated obstacle-avoidance failure"
        )
    if regressed:
        inconclusive.append(
            f"simulation time moved backward on {len(regressed)}/{len(rows)} recorded rows "
            f"(first at t_sim_s={regressed[0]['t_sim_s']:.2f}) - affected evidence is untrustworthy"
        )

    gt_all_sampled = all(r["gt_sampled"] for r in rows) if ground_truth_requested else False
    if not ground_truth_requested:
        inconclusive.append(
            "clearance is odom-derived only (--ground-truth not used); odometry is known in "
            "this project to drift under turning (see prior SLAM-mapping findings) - clearance "
            "numbers are indicative, not proof of collision-free motion, so a collision-safety "
            "PASS claim cannot be made from this run"
        )
    elif not gt_all_sampled:
        failed = [r for r in rows if not r["gt_sampled"]]
        errors = sorted({r["gt_error"] for r in failed if r["gt_error"]})
        inconclusive.append(
            f"ground truth sampling failed on {len(failed)}/{len(rows)} rows "
            f"(reasons: {errors[:3]}{', ...' if len(errors) > 3 else ''}) - incomplete "
            f"ground-truth evidence, required for a collision-safety PASS claim"
        )

    delayed = sum(1 for r in rows if r["sample_delayed"])
    if delayed:
        reasons.append(f"{delayed}/{len(rows)} samples had a tick far slower than the target rate "
                        f"(see tick_duration_s in the CSV) - sampling cadence was not uniform")

    tail = rows[-max(1, len(rows) // 10):]
    final_cmd_zero = all(
        abs(r["cmd_vel_linear_x"] or 0.0) < 1e-3 and abs(r["cmd_vel_angular_z"] or 0.0) < 1e-3
        for r in tail
    )
    reasons.append(
        "robot commanded to stop by end of run (inspect odom/gt trajectory in the CSV to tell "
        "apart: obstacle cleared and stopped, vs. no-safe-direction stop - a zero command with "
        "fresh sensor data is not automatically a failure)"
        if final_cmd_zero else
        "robot still commanding motion at end of run - duration may be too short to observe a "
        "terminal outcome"
    )

    if inconclusive:
        return INCONCLUSIVE, inconclusive + reasons
    return PASS, reasons


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path, verdict, reasons):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write(f"VERDICT: {verdict}\n\n")
        for r in reasons:
            f.write(f"- {r}\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    default_dir = os.path.join(os.path.dirname(__file__), "results")
    default_name = f"baseline_run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    parser.add_argument("--output", default=os.path.join(default_dir, default_name),
                         help="CSV output path (default: timestamped, under scripts/validation/results/). "
                              "A sibling <output_without_ext>.summary.txt is also written.")
    parser.add_argument("--duration", type=float, default=30.0,
                         help="Recording duration in seconds, wall clock (default 30).")
    parser.add_argument("--rate", type=float, default=10.0,
                         help="Sampling rate in Hz, matches the LiDAR's 10 Hz update_rate (default 10).")
    parser.add_argument("--prerun-timeout", type=float, default=5.0,
                         help="Seconds to wait for /scan, /odom, /cmd_vel before giving up (default 5).")
    parser.add_argument("--allow-scan-timeout-fallback", action="store_true",
                         help="Opt-in only, disabled by default: if vfh_plus_node's live scan_timeout "
                              "parameter is unavailable, unset, wrongly typed, non-finite, zero, or "
                              "negative, permit falling back to --scan-timeout-fallback instead of "
                              "failing preflight. The fallback and the reason it was needed are always "
                              "recorded in the summary - never silent.")
    parser.add_argument("--scan-timeout-fallback", type=float, default=FALLBACK_SCAN_TIMEOUT_S,
                         help="Only used when --allow-scan-timeout-fallback is also passed "
                              f"(default {FALLBACK_SCAN_TIMEOUT_S}, matching vfh_plus_node's declared "
                              f"default). Without --allow-scan-timeout-fallback, an invalid live "
                              f"parameter fails preflight instead of using this value.")
    parser.add_argument("--ground-truth", action="store_true",
                         help="Selects ground-truth validation mode: polls Gazebo ground-truth pose via "
                              "`gz model -p` each tick, and clearance comes only from that (never a "
                              "silent odometry fallback on a failed query). GROUND TRUTH IS REQUIRED "
                              "FOR A PASS IN THIS BASELINE VALIDATION MODE - without this flag, or with "
                              "any row's ground-truth sampling incomplete, the run is capped at "
                              "INCONCLUSIVE no matter how clean everything else looks. Without this "
                              "flag, clearance is odom-derived instead (odometry-only mode).")
    parser.add_argument("--gt-timeout", type=float, default=2.0,
                         help="Timeout for each `gz model -p` subprocess call, seconds (default 2).")
    parser.add_argument("--cmd-vel-fresh-timeout", type=float, default=CMD_VEL_FRESH_TIMEOUT_S,
                         help="Max age, in seconds, before /cmd_vel is considered stale (default "
                              f"{CMD_VEL_FRESH_TIMEOUT_S} - 3x vfh_plus_node's own ~0.1s watchdog/scan-"
                              f"driven republish period; see the module docstring's CMD_VEL FRESHNESS "
                              f"section for why this is not reused from --scan-timeout-fallback).")
    parser.add_argument("--max-rows", type=int, default=100000, help="Safety cap on recorded rows.")
    parser.add_argument("--force", action="store_true",
                         help="Allow overwriting an existing output path (default: refuse).")
    return parser.parse_args()


def resolve_output_paths(args):
    csv_path = args.output
    summary_path = os.path.splitext(csv_path)[0] + ".summary.txt"
    if not args.force:
        for path in (csv_path, summary_path):
            if os.path.exists(path):
                sys.exit(f"refusing to overwrite existing output file: {path} "
                         f"(use --force or choose a different --output)")
    return csv_path, summary_path


def main():
    args = parse_args()
    csv_path, summary_path = resolve_output_paths(args)
    print(LAUNCH_REMINDER)

    rclpy.init()
    node = ValidationRecorder()
    rows = []
    interrupted = False
    try:
        prerun_verdict, prerun_notes, scan_timeout = run_prerun_checks(node, args)
        if prerun_verdict == INCONCLUSIVE:
            write_csv(csv_path, [])
            write_summary(summary_path, INCONCLUSIVE, prerun_notes)
            print(f"INCONCLUSIVE (prerun failed):")
            for n in prerun_notes:
                print(f"  - {n}")
            return EXIT_CODES[INCONCLUSIVE]

        start = time.monotonic()
        prev_t_sim_s = None
        try:
            while time.monotonic() - start < args.duration and len(rows) < args.max_rows:
                tick_start = time.monotonic()
                rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
                row = sample_row(node, args, scan_timeout, prev_t_sim_s)
                tick_duration = time.monotonic() - tick_start
                row["tick_duration_s"] = tick_duration
                row["sample_delayed"] = tick_duration > 2.0 * (1.0 / args.rate)
                rows.append(row)
                prev_t_sim_s = row["t_sim_s"]
        except KeyboardInterrupt:
            interrupted = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    write_csv(csv_path, rows)
    verdict, verdict_reasons = compute_verdict(rows, args.ground_truth, interrupted)
    all_notes = prerun_notes + verdict_reasons
    write_summary(summary_path, verdict, all_notes)
    print(f"{verdict}")
    for n in all_notes:
        print(f"  - {n}")
    print(f"\nSee {csv_path} and {summary_path}")
    return EXIT_CODES[verdict]


if __name__ == "__main__":
    sys.exit(main())
