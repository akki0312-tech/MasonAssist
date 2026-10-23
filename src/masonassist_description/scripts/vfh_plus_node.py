#!/usr/bin/env python3
"""ROS 2 wrapper around VFHPlusCore - the standalone, ROS-free VFH+ core.

This node owns ONLY: subscriptions, LaserScan->robot-frame conversion via
TF, watchdog/safety gating, ROS parameters, and converting the core's
selected direction into a Twist. VFHPlusCore remains the sole decision-
making algorithm - nothing here reimplements or overrides its logic.

GOAL SOURCE (v2 - supersedes the old v1 fixed-zero contract below): this
node now accepts a live goal position on /goal_pose (geometry_msgs/Point,
x/y in the /odom frame). Each scan tick, if a goal is active, goal_direction
is recomputed from the live /odom pose and fed into VFHPlusCore.params
(the dataclass field is mutated in place, not reconstructed, so the core's
own hysteresis state - previous_direction, binary_state - is preserved
across goal updates exactly as it is across ordinary ticks). This is the
"external goal source" the original v1 docstring said did not exist yet -
nothing about the core's own decision logic changes, only what feeds its
existing goal_direction parameter.

HEADING DECISION (unchanged from v1): current_heading_error stays fixed at
0.0. The core's convention is "0 rad = straight ahead in the robot's OWN
current frame, refreshed every call" - current_heading_error is a distinct
parameter from goal_direction (see core module docstring: both reduce to
the same reference only because neither existed before) and is still
trivially 0 by construction; no heading-error source is introduced here.

LASER FRAME HANDLING: the LiDAR mount offset (0.30 m / 0.19 m) is never
hard-coded here. Each scan's points are converted from its own
header.frame_id (laser_frame) into base_link using a live tf2 lookup, so
this node has no dependency on the exact mount geometry at all.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import sys
import os
# realpath: the installed node is a symlink, and worker_mover.py (needed by
# safety_zones) lives only next to the real source file.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import json  # noqa: E402
from safety_zones import SafetyGate, unmapped_points  # noqa: E402
from vfh_plus_core import VFHParams, VFHPlusCore, VFHState, wrap_to_pi  # noqa: E402
from robot_config import ROBOT  # noqa: E402


CURRENT_HEADING_ERROR = 0.0  # unchanged from v1, see module docstring
GOAL_TOLERANCE_M = 0.3
RECOVERY_BLOCKED_S = 3.0   # standard VFH-family recovery: rotate in place
RECOVERY_TURN_RATE = 0.3   # to re-scan for an opening after being stuck this long

# Stall reporting: a goal that is permanently obstructed (e.g. placed inside
# solid geometry) correctly never reaches 'reached' - the robot cycles
# blocked/recovering indefinitely, which is the right MOTION behavior (a
# real blockage, like a worker walking past, can clear on its own, so
# retrying forever is appropriate and is left unchanged here). But observed
# directly: every normal, resolving blockage in this project's own test
# goals cleared within ~10s, while a genuinely unreachable goal (sent to a
# point inside wall_north's collision) stayed blocked/recovering for 25s+
# with no change in mission_status to distinguish the two cases - an
# operator watching the dashboard cannot tell "about to resolve" from
# "will never resolve" from status alone. This adds ONLY a distinct status
# string once continuously blocked well beyond that normal range; it does
# not change the recovery motion/retry behavior at all.
MISSION_TIMEOUT_BLOCKED_S = 25.0

# Escape from inside the safety envelope. VFHPlusCore.is_direction_safe
# rejects EVERY direction while any point is within r_effective (by design,
# see its docstring and test_obstacle_inside_r_effective_blocks_every_candidate),
# so once the robot drifts that close to an obstacle - observed live at a
# partition corner, 0.45 m from base_link - no direction is ever accepted and
# rotate-in-place recovery cannot help (rotation does not change the distance).
# The robot then only crept out by accident over tens of seconds. This backs
# it straight away from the nearest point, slowly, only if that short path is
# clear, until it is outside r_effective and normal VFH+ selection resumes.
PIVOT_THRESHOLD_RAD = math.radians(45)   # see _compute_twist
TURN_LATCH_RAD = math.radians(135)

ESCAPE_SPEED = 0.08          # m/s, deliberately slow
ESCAPE_LOOKAHEAD_M = 0.4     # swept-path length checked before each escape tick


def escape_linear_velocity(ranges, angles, r_robot, range_min):
    """Signed linear.x that moves the robot straight away from its nearest
    valid return (reverse if that return is in the front half, forward if in
    the rear half), or None if the swept path in that direction is not clear.
    ranges/angles are robot-frame (base_link), as fed to VFHPlusCore."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min)
    if not np.any(valid):
        return None
    r, a = ranges[valid], angles[valid]
    nearest = a[int(np.argmin(r))]
    direction = math.pi if abs(wrap_to_pi(nearest)) < math.pi / 2 else 0.0
    for d, ang in zip(r, a):
        delta = wrap_to_pi(ang - direction)
        along = d * math.cos(delta)
        lateral = d * abs(math.sin(delta))
        if 0.0 < along < r_robot + ESCAPE_LOOKAHEAD_M and lateral < r_robot:
            return None
    return -ESCAPE_SPEED if direction == math.pi else ESCAPE_SPEED

# Self-occlusion mask: the gpu_lidar sensor sweeps a full 360 deg from its
# mount point (0.30 m forward of base_link). At that mount position, the
# left_track/right_track bodies intrude into the sensor's own field of
# view near laser-frame bearing +-100 deg, always returning the robot's
# own structure at its sensor-floor range. Confirmed directly against
# live /scan data (reproducible across robot orientations, since it is a
# fixed body-relative geometric relationship, not a real external
# obstacle): two bands at [-104.8, -97.8] deg and [97.8, 104.8] deg,
# min range 0.10-0.19 m. VFHPlusCore correctly treats any point this
# close (inside r_effective=0.5975 m) as maximally dangerous and spreads
# a full 180 deg "occupied" hemisphere around it (enlargement_half_angle's
# d<=r_effective case) - exactly correct for a REAL obstacle that close,
# but these two bands are not real obstacles. Left unmasked, they
# blanket nearly the entire 360 deg histogram (verified: 67/72 sectors
# occupied, the one nominally-free valley rejected, BLOCKED reported even
# in open space) regardless of the actual external environment. Padded
# 1 deg beyond the measured extent for margin. VFHPlusCore itself is not
# modified - this only prevents two known-false rays from ever reaching
# it, the same way an out-of-range reading already would not.
SELF_OCCLUSION_BANDS_DEG = tuple(tuple(b) for b in ROBOT['lidar']['self_occlusion_bands_deg'])


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VFHPlusNode(Node):

    def __init__(self):
        super().__init__('vfh_plus_node')

        core_defaults = VFHParams()

        # ---- Parameters -----------------------------------------------
        # Geometry fact, not a tuning knob (derived from the real chassis
        # footprint 0.725 x 0.525 m) - exposed as a parameter for
        # flexibility only, default taken from the core, not re-invented.
        self.declare_parameter('robot_radius', ROBOT['navigation']['robot_radius_m'])
        # Initial/tunable, unvalidated against real data - see HANDOFF.md
        # and the VFH+ design-study history for the full discussion.
        self.declare_parameter('safety_margin', ROBOT['navigation']['safety_margin_m'])
        self.declare_parameter('d_max', core_defaults.d_max)
        # Currently UNVALIDATED against real histogram distributions.
        self.declare_parameter('threshold_high', core_defaults.threshold_high)
        self.declare_parameter('threshold_low', core_defaults.threshold_low)
        self.declare_parameter('smoothing_window', core_defaults.smoothing_window)
        self.declare_parameter('weight_goal', core_defaults.weight_goal)
        self.declare_parameter('weight_heading', core_defaults.weight_heading)
        self.declare_parameter('weight_previous', core_defaults.weight_previous)

        # Command-generation parameters (not part of VFHParams - these
        # belong to this node's Twist-conversion layer, kept separate from
        # the core's own decision logic per the task's explicit instruction).
        # forward_speed_max / max_angular_speed match the already-tested
        # propulsion baseline values used elsewhere in this project
        # (simple_obstacle_avoidance.py's forward_speed/turn_speed) - not
        # newly invented. steering_gain and scan_timeout are genuinely new
        # for this node and are NOT validated - conservative, round
        # starting points only.
        self.declare_parameter('forward_speed_max', ROBOT['navigation']['forward_speed_max'])
        self.declare_parameter('max_angular_speed', ROBOT['navigation']['max_angular_speed'])
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('scan_timeout', 0.5)

        params = VFHParams(
            r_robot=self.get_parameter('robot_radius').value,
            safety_margin=self.get_parameter('safety_margin').value,
            d_max=self.get_parameter('d_max').value,
            range_min=core_defaults.range_min,
            range_max=core_defaults.range_max,
            sector_width_deg=core_defaults.sector_width_deg,
            smoothing_window=self.get_parameter('smoothing_window').value,
            threshold_high=self.get_parameter('threshold_high').value,
            threshold_low=self.get_parameter('threshold_low').value,
            weight_goal=self.get_parameter('weight_goal').value,
            weight_heading=self.get_parameter('weight_heading').value,
            weight_previous=self.get_parameter('weight_previous').value,
            goal_direction=0.0,  # overwritten live each tick once a goal is active
        )
        self.core = VFHPlusCore(params)

        self.forward_speed_max = self.get_parameter('forward_speed_max').value
        self._last_turn_sign = 0
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # ---- TF ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Watchdog state ---------------------------------------------
        self.last_scan_time = None
        self.last_odom_time = None
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        # ---- Goal/mission state ------------------------------------------
        self.goal = None            # (x, y) in /odom frame, or None if no mission
        self.mission_active = False  # False once goal reached, True while driving
        self.paused = False
        self.estopped = False
        self.status = 'idle'
        self.blocked_since = None  # rclpy Time when BLOCKED state first began, or None

        # ---- ROS interfaces ----------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.safety_pub = self.create_publisher(String, '/safety_status', 10)
        self.safety_gate = SafetyGate()
        self._safety_published = False
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # pose in the site/map frame: /robot_pose (map_pose_publisher) in the
        # full stack; plain /odom when run on its own
        pose_topic = self.declare_parameter('pose_topic', '/odom').value
        self.odom_sub = self.create_subscription(Odometry, pose_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal_pose', self.goal_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/mission_cmd', self.mission_cmd_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('vfh_plus_node started (r_effective=%.4f m)' % params.r_effective)

    # ---------------------------------------------------------------------

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom_time = self.get_clock().now()
        p = msg.pose.pose.position
        self.odom_x, self.odom_y = p.x, p.y
        self.odom_yaw = _yaw_of(msg.pose.pose.orientation)

    def goal_callback(self, msg: Point) -> None:
        self.goal = (msg.x, msg.y)
        self.mission_active = True
        self.paused = False
        self.get_logger().info(f'New goal received: x={msg.x:.3f} y={msg.y:.3f}')

    def mission_cmd_callback(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == 'cancel':
            self.goal = None
            self.mission_active = False
            self.paused = False
            self._publish_zero()
        elif cmd == 'pause':
            self.paused = True
            self._publish_zero()
        elif cmd == 'resume':
            self.paused = False
        elif cmd == 'estop':
            self.estopped = True
            self._publish_zero()
        elif cmd == 'clear_estop':
            # Verified directly (Phase 2 baseline check): clearing estopped
            # alone let the next scan tick immediately resume driving
            # toward whatever goal was still active - no separate operator
            # action required. Nothing in this project documents that as
            # intended, and an e-stop clear silently restarting motion is
            # a real safety gap. Mirrors the existing, already-correct
            # pause/resume contract exactly: clearing the stop leaves the
            # robot paused, same as it would be after 'pause', so an
            # explicit 'resume' is required to actually move again.
            self.estopped = False
            self.paused = True
        else:
            self.get_logger().warn(f'Unknown /mission_cmd: {cmd!r}')

    def _publish_status(self, status: str) -> None:
        self.status = status
        self.status_pub.publish(String(data=status))

    def scan_callback(self, msg: LaserScan) -> None:
        self._safety_published = False
        self._scan_step(msg)
        if not self._safety_published:
            # no motion commanded this tick (idle, paused, e-stopped, reached,
            # blocked...) - say so, so the dashboard never shows a stale STOP
            self._publish_safety('inactive', None, 0)

    def _scan_step(self, msg: LaserScan) -> None:
        self.last_scan_time = self.get_clock().now()

        if self.estopped:
            self.blocked_since = None
            self._publish_zero()
            self._publish_status('estopped')
            return

        if self.last_odom_time is None or \
                (self.get_clock().now() - self.last_odom_time) > Duration(seconds=self.scan_timeout):
            self.get_logger().warn('odometry unavailable/stale - publishing zero Twist', throttle_duration_sec=2.0)
            self._publish_zero()
            self._publish_status('no_data')
            return

        if self.goal is None:
            self.blocked_since = None
            self._publish_zero()
            self._publish_status('idle')
            return

        if self.paused:
            self._publish_zero()
            self._publish_status('paused')
            return

        dx = self.goal[0] - self.odom_x
        dy = self.goal[1] - self.odom_y
        distance = math.hypot(dx, dy)
        if distance < GOAL_TOLERANCE_M:
            self.mission_active = False
            self.blocked_since = None
            self._publish_zero()
            self._publish_status('reached')
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f'TF lookup failed ({msg.header.frame_id} -> base_link): {ex} '
                                    '- publishing zero Twist', throttle_duration_sec=2.0)
            self._publish_zero()
            self._publish_status('no_data')
            return

        robot_ranges, robot_angles = self._scan_to_robot_frame(msg, tf)

        # Goal direction, recomputed every tick from the live /odom pose and
        # fed into the core's existing goal_direction field in place (see
        # module docstring GOAL SOURCE section) - the core's own hysteresis
        # state is never reset by this.
        goal_bearing_world = math.atan2(dy, dx)
        self.core.params.goal_direction = wrap_to_pi(goal_bearing_world - self.odom_yaw)
        # don't let obstacles beyond where we will stop veto the approach
        self.core.params.horizon = distance + self.core.params.r_effective

        result = self.core.process(robot_ranges, robot_angles, current_heading_error=CURRENT_HEADING_ERROR)

        if result.state != VFHState.OK:
            now = self.get_clock().now()
            if result.state == VFHState.BLOCKED:
                if self.blocked_since is None:
                    self.blocked_since = now
                p = self.core.params
                if result.min_obstacle_distance is not None and \
                        result.min_obstacle_distance <= p.r_effective:
                    linear = escape_linear_velocity(robot_ranges, robot_angles, p.r_robot, p.range_min)
                    if linear is not None:
                        twist = Twist()
                        twist.linear.x = linear
                        timed_out = (now - self.blocked_since) > Duration(seconds=MISSION_TIMEOUT_BLOCKED_S)
                        self._publish_motion(twist, 'stalled' if timed_out else 'recovering',
                                             robot_ranges, robot_angles)
                        return
                if (now - self.blocked_since) > Duration(seconds=RECOVERY_BLOCKED_S):
                    # Standard VFH-family recovery: no safe valley has been
                    # found for RECOVERY_BLOCKED_S seconds straight - rotate
                    # in place (never drive blind) so the next scan sees new
                    # geometry. This is a real, sensor-gated behavior, not a
                    # scripted motion: it only ever runs while result.state
                    # stays BLOCKED, and the very next OK tick ends it.
                    twist = Twist()
                    twist.angular.z = RECOVERY_TURN_RATE
                    timed_out = (now - self.blocked_since) > Duration(seconds=MISSION_TIMEOUT_BLOCKED_S)
                    self._publish_motion(twist, 'stalled' if timed_out else 'recovering',
                                         robot_ranges, robot_angles)
                    return
            else:
                self.blocked_since = None
            self.get_logger().info(f'VFH+ state={result.state.value} - publishing zero Twist',
                                    throttle_duration_sec=2.0)
            self._publish_zero()
            self._publish_status(result.state.value)
            return

        self.blocked_since = None
        self._publish_motion(self._compute_twist(result.selected_direction), 'navigating',
                             robot_ranges, robot_angles)

    def _publish_motion(self, twist: Twist, status: str, robot_ranges, robot_angles) -> None:
        """Single choke point for every non-zero motion command: the human-
        aware SafetyGate (safety_zones.py) limits it by the nearest unmapped
        return in the direction of travel. A gate stop publishes zero and
        status 'yielding'; 'slow' keeps the normal status. Zero commands
        (e-stop, pause, idle, watchdog) never come through here - stopping is
        always allowed."""
        un_r, un_a = unmapped_points(robot_ranges, robot_angles,
                                     (self.odom_x, self.odom_y, self.odom_yaw),
                                     self.core.params.range_min)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        lin, ang, zone, nearest = self.safety_gate.filter(
            twist.linear.x, twist.angular.z, un_r, un_a, now_s)
        out = Twist()
        out.linear.x = lin
        out.angular.z = ang
        self.cmd_vel_pub.publish(out)
        self._publish_status('yielding' if zone == 'stop' else status)
        self._publish_safety(zone, nearest, int(un_r.size))

    def _publish_safety(self, zone: str, nearest, unmapped: int) -> None:
        self._safety_published = True
        self.safety_pub.publish(String(data=json.dumps({
            'zone': zone,
            'nearest_m': None if nearest is None else round(nearest, 2),
            'unmapped': unmapped,
        })))

    def watchdog_callback(self) -> None:
        """Independent of scan_callback: if scans stop arriving entirely,
        nothing would otherwise re-publish and a stale non-zero command
        would sit on /cmd_vel indefinitely. This is the fail-safe for that
        case - it only ever publishes zero, never a normal command."""
        now = self.get_clock().now()
        stale_scan = self.last_scan_time is None or (now - self.last_scan_time) > Duration(seconds=self.scan_timeout)
        stale_odom = self.last_odom_time is None or (now - self.last_odom_time) > Duration(seconds=self.scan_timeout)
        if stale_scan or stale_odom:
            self._publish_zero()

    # ---------------------------------------------------------------------

    def _scan_to_robot_frame(self, msg: LaserScan, tf) -> tuple:
        """Convert every (range, angle) in msg (its own header.frame_id,
        e.g. laser_frame) into robot-frame (base_link) polar coordinates,
        using the live TF transform - no hard-coded mount offset.

        Two fixes applied here, both confirmed directly against live data
        (see SELF_OCCLUSION_BANDS_DEG above for the first):
        1. Known self-occlusion bands are masked to +inf ("no return",
           the same convention the sensor already uses for out-of-range
           rays) before any other processing, so they never reach
           VFHPlusCore as if they were real obstacles.
        2. Only finite ranges participate in the rotation math. Doing the
           rotation on the full array (including the sensor's own
           out-of-range +-inf entries) produced 0*inf = NaN whenever this
           mount's yaw component was exactly zero (confirmed: "invalid
           value encountered in multiply" RuntimeWarning at this robot's
           actual mount orientation). Non-finite entries are written
           through as +inf in the output instead, so VFHPlusCore's own
           filter_valid (np.isfinite) still excludes them - unchanged
           behavior for every genuinely out-of-range ray.
        """
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        angles_deg = np.degrees(angles)

        for lo, hi in SELF_OCCLUSION_BANDS_DEG:
            ranges = np.where((angles_deg >= lo) & (angles_deg <= hi), np.inf, ranges)

        finite = np.isfinite(ranges)
        f_ranges = ranges[finite]
        f_angles = angles[finite]

        lx = f_ranges * np.cos(f_angles)
        ly = f_ranges * np.sin(f_angles)

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        bx = t.x + cos_yaw * lx - sin_yaw * ly
        by = t.y + sin_yaw * lx + cos_yaw * ly

        robot_ranges = np.full(len(ranges), np.inf)
        robot_angles = np.zeros(len(ranges))
        robot_ranges[finite] = np.hypot(bx, by)
        robot_angles[finite] = np.arctan2(by, bx)
        return robot_ranges, robot_angles

    def _compute_twist(self, selected_direction: float) -> Twist:
        """Command-generation layer, deliberately separate from the core's
        own decision logic. angular.z is a clipped proportional response to
        the selected heading error; linear.x is scaled by cos(error) so it
        falls to zero (favoring a pure pivot, not blind forward motion) as
        the required turn approaches +-90 deg, and stays zero beyond that -
        no additional arbitrary tuning beyond the exposed parameters.

        Two corrections, both from live data with ground-truth odometry
        (the robot orbited a goal at 0.85-1.45 m for minutes, angular.z
        flipping +-0.3 every second):
        - Pivot first: no forward motion while |error| > PIVOT_THRESHOLD_RAD.
          The tracks realise only part of the commanded turn while also
          driving, so cos-scaled forward speed at large errors gave a ~1 m
          effective turning radius and the goal stayed inside it.
        - Turn-direction latch: near +-180 deg the selected direction flips
          sign tick to tick (+175 / -175), so keep turning the way we were
          already turning instead of reversing every tick."""
        twist = Twist()
        error = wrap_to_pi(selected_direction)
        if abs(error) > TURN_LATCH_RAD and self._last_turn_sign != 0:
            angular = self._last_turn_sign * self.max_angular_speed
        else:
            angular = self.steering_gain * error
        twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        if twist.angular.z != 0.0:
            self._last_turn_sign = 1 if twist.angular.z > 0 else -1
        if abs(error) <= PIVOT_THRESHOLD_RAD:
            twist.linear.x = self.forward_speed_max * math.cos(error)
        return twist

    def _publish_zero(self) -> None:
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VFHPlusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
