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
and this script does not attempt that reconciliation.
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

FALLBACK_SCAN_TIMEOUT_S = 0.5  # only used if the live node's own parameter can't be queried

CSV_FIELDS = [
    "t_sim_s", "t_wall_s",
    "scan_age_s", "scan_min_range_m",
    "odom_age_s", "odom_x", "odom_y", "odom_yaw_deg",
    "cmd_vel_age_s", "cmd_vel_linear_x", "cmd_vel_angular_z",
    "gt_sampled", "gt_x", "gt_y", "gt_yaw_deg", "gt_sample_wall_latency_s",
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


def box_clearance(px, py, cx, cy, half_extent, r_robot):
    """Planar distance from a circle of radius r_robot centered at (px, py)
    to the nearest surface of an axis-aligned square, minus r_robot. See the
    module docstring's COLLISION CLEARANCE CAVEAT before interpreting a
    negative result as a proven collision."""
    dx = max(abs(px - cx) - half_extent, 0.0)
    dy = max(abs(py - cy) - half_extent, 0.0)
    return math.hypot(dx, dy) - r_robot


def query_ground_truth_pose(model_name, timeout_s):
    """Blocking `gz model -p` subprocess call (same parsing approach used in
    this project's earlier SLAM-mapping scratchpad scripts). Returns
    (x, y, yaw_rad, wall_latency_s), or None on failure/timeout/parse error."""
    t0 = time.monotonic()
    try:
        out = subprocess.run(
            ["gz", "model", "-m", model_name, "-p"],
            capture_output=True, text=True, timeout=timeout_s,
        ).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return None
    latency = time.monotonic() - t0
    nums = [list(map(float, m.split())) for m in re.findall(r"\[([-\d\. e]+)\]", out)]
    if len(nums) < 2:
        return None
    x, y = nums[-2][0], nums[-2][1]
    yaw = nums[-1][2]
    return x, y, yaw, latency


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

    def query_live_scan_timeout(self, fallback, timeout_s=3.0):
        """Fetch vfh_plus_node's actual scan_timeout parameter via its own
        get_parameters service, so freshness is checked against the live
        running configuration rather than a hardcoded duplicate of its
        default. Falls back to `fallback` (loudly, via the returned bool)
        if the service is unavailable."""
        client = self.create_client(GetParameters, f"/{EXPECTED_CONTROLLER_NODE}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                return fallback, False
            future = client.call_async(GetParameters.Request(names=["scan_timeout"]))
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            result = future.result() if future.done() else None
            if result is None or not result.values:
                return fallback, False
            return result.values[0].double_value, True
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

    scan_timeout, is_live = node.query_live_scan_timeout(args.scan_timeout_fallback)
    reasons.append(
        f"scan_timeout={scan_timeout}s ({'queried live from ' + EXPECTED_CONTROLLER_NODE if is_live else 'FALLBACK ASSUMED - live query failed'})"
    )

    scan_age, odom_age = node.scan_age_s(), node.odom_age_s()
    if scan_age is None or scan_age > scan_timeout:
        reasons.append(f"/scan stale or unavailable (age={scan_age})")
    if odom_age is None or odom_age > scan_timeout:
        reasons.append(f"/odom stale or unavailable (age={odom_age})")

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


def sample_row(node, args):
    now = node.get_clock().now()
    row = dict.fromkeys(CSV_FIELDS)
    row["t_sim_s"] = now.nanoseconds / 1e9
    row["t_wall_s"] = time.time()
    row["scan_age_s"] = node.scan_age_s()
    row["scan_min_range_m"] = node.scan_min_range()
    row["odom_age_s"] = node.odom_age_s()
    row["cmd_vel_age_s"] = node.cmd_vel_age_s()
    row["gt_sampled"] = False

    odom = node.odom_xy_yaw()
    if odom:
        row["odom_x"], row["odom_y"], yaw = odom
        row["odom_yaw_deg"] = math.degrees(yaw)
    if node.latest_cmd_vel is not None:
        row["cmd_vel_linear_x"] = node.latest_cmd_vel.linear.x
        row["cmd_vel_angular_z"] = node.latest_cmd_vel.angular.z

    clearance_xy, clearance_source = None, None
    if args.ground_truth:
        gt = query_ground_truth_pose(SPAWN_MODEL_NAME, args.gt_timeout)
        if gt:
            x, y, yaw, latency = gt
            row["gt_sampled"] = True
            row["gt_x"], row["gt_y"], row["gt_yaw_deg"] = x, y, math.degrees(yaw)
            row["gt_sample_wall_latency_s"] = latency
            clearance_xy, clearance_source = (x, y), "ground_truth"
    if clearance_xy is None and odom:
        clearance_xy, clearance_source = (row["odom_x"], row["odom_y"]), "odom (not independently verified)"

    if clearance_xy:
        row["geom_clearance_m"] = box_clearance(
            clearance_xy[0], clearance_xy[1],
            OBSTACLE_CENTER_XY[0], OBSTACLE_CENTER_XY[1],
            OBSTACLE_HALF_EXTENT, R_ROBOT,
        )
        row["clearance_source"] = clearance_source
    return row


def compute_verdict(rows, scan_timeout, ground_truth_requested, interrupted):
    if not rows:
        return INCONCLUSIVE, ["no rows recorded during the run window"]

    reasons = []
    stale = sum(
        1 for r in rows
        if (r["scan_age_s"] is not None and r["scan_age_s"] > scan_timeout)
        or (r["odom_age_s"] is not None and r["odom_age_s"] > scan_timeout)
    )
    cmd_vel_ever_nonzero = any(
        abs(r["cmd_vel_linear_x"] or 0.0) > 1e-3 or abs(r["cmd_vel_angular_z"] or 0.0) > 1e-3
        for r in rows
    )
    clearances = [r["geom_clearance_m"] for r in rows if r["geom_clearance_m"] is not None]
    min_clearance = min(clearances) if clearances else None
    tail = rows[-max(1, len(rows) // 10):]
    final_cmd_zero = all(
        abs(r["cmd_vel_linear_x"] or 0.0) < 1e-3 and abs(r["cmd_vel_angular_z"] or 0.0) < 1e-3
        for r in tail
    )
    gt_all_sampled = all(r["gt_sampled"] for r in rows) if ground_truth_requested else False

    if interrupted:
        reasons.append("run was interrupted (Ctrl+C) before its configured duration elapsed")

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

    verdict = PASS
    if not ground_truth_requested:
        reasons.append(
            "clearance is odom-derived only (--ground-truth not used); odometry is known in "
            "this project to drift under turning (see prior SLAM-mapping findings) - clearance "
            "numbers are indicative, not proof of collision-free motion"
        )
        verdict = INCONCLUSIVE
    elif not gt_all_sampled:
        reasons.append(
            "ground truth was requested but failed to sample on at least one row (gz model "
            "query failed/timed out) - those rows fell back to odom-derived clearance"
        )
        verdict = INCONCLUSIVE

    if stale:
        reasons.append(f"{stale}/{len(rows)} recorded samples had scan/odom age over scan_timeout")
    reasons.append(
        "robot commanded to stop by end of run (inspect odom/gt trajectory in the CSV to tell "
        "apart: obstacle cleared and stopped, vs. no-safe-direction stop)"
        if final_cmd_zero else
        "robot still commanding motion at end of run - duration may be too short to observe a "
        "terminal outcome"
    )
    return verdict, reasons


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
    parser.add_argument("--scan-timeout-fallback", type=float, default=FALLBACK_SCAN_TIMEOUT_S,
                         help="Used only if vfh_plus_node's live scan_timeout parameter can't be queried "
                              f"(default {FALLBACK_SCAN_TIMEOUT_S}, matching its declared default).")
    parser.add_argument("--ground-truth", action="store_true",
                         help="Also poll Gazebo ground-truth pose via `gz model -p` each tick. Without "
                              "this flag, clearance is odom-derived only and the verdict is capped at "
                              "INCONCLUSIVE for collision-safety claims.")
    parser.add_argument("--gt-timeout", type=float, default=2.0,
                         help="Timeout for each `gz model -p` subprocess call, seconds (default 2).")
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
        try:
            while time.monotonic() - start < args.duration and len(rows) < args.max_rows:
                rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
                rows.append(sample_row(node, args))
        except KeyboardInterrupt:
            interrupted = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    write_csv(csv_path, rows)
    verdict, verdict_reasons = compute_verdict(rows, scan_timeout, args.ground_truth, interrupted)
    all_notes = prerun_notes + verdict_reasons
    write_summary(summary_path, verdict, all_notes)
    print(f"{verdict}")
    for n in all_notes:
        print(f"  - {n}")
    print(f"\nSee {csv_path} and {summary_path}")
    return EXIT_CODES[verdict]


if __name__ == "__main__":
    sys.exit(main())
