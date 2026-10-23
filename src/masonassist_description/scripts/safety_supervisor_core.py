#!/usr/bin/env python3
"""ROS-free safety supervisor logic for MasonAssist.

Sits between navigation and the robot: navigation publishes /cmd_vel_nav,
the supervisor decides what actually goes to /cmd_vel, every tick, even if
navigation has stopped publishing. Measured before this existed: with the
navigation node frozen mid-drive, the robot kept driving blind on its last
command for over 3 m - nothing else could stop it.

Every condition below forces zero (or blocks one linear direction), each
with a named reason, independently of the navigation node's own logic.
"""

import math

import numpy as np

from robot_config import ROBOT

CMD_TIMEOUT_S = 0.3
SENSOR_TIMEOUT_S = 0.5
LINK_TIMEOUT_S = 2.0
# Command/sensor ages are judged in sim time (= wall time on a real robot):
# with the Gazebo GUI open the whole simulation hiccups, so /scan and the
# commands that follow it can arrive 0.3-0.57 s apart in wall time while
# only ~0.15 s of sim time - the only time the robot can move in - passed.
# Wall time is still checked against this looser backstop, so a frozen
# clock or simulator still stops the robot.
WALL_BACKSTOP_S = 1.0
MAX_LINEAR = ROBOT['safety']['max_linear']      # m/s
MAX_ANGULAR = ROBOT['safety']['max_angular']    # rad/s

# collision guard: robot body in base_link (chassis 0.725 x 0.525 m, tracks
# out to ~0.43 m each side), plus a stopping margin in the direction of travel
BODY_FRONT_M = ROBOT['body']['front_m']
BODY_REAR_M = ROBOT['body']['rear_m']
BODY_HALF_WIDTH_M = ROBOT['body']['half_width_m']
GUARD_MARGIN_M = ROBOT['safety']['guard_margin_m']
LIDAR_X_OFFSET = ROBOT['lidar']['x_offset_m']
LIDAR_MIN_RANGE_M = ROBOT['lidar']['min_range_m']   # sensor <min>; returns inside the body
                                                    # rectangle are excluded by the lane test below
SELF_BANDS_DEG = tuple(tuple(b) for b in ROBOT['lidar']['self_occlusion_bands_deg'])

# Tilt guard (IMU). Limits from the simulated terrain tests (docs/terrain):
# up to 25 kg the robot held and drove on everything tested (35 deg static),
# with 50 kg (load centre ~0.43 m high) it tipped braking down 30 deg and
# standing on 35 deg, but held and braked fine at 20 deg (tilt reads 20.4 deg
# there), so the 50 kg limit sits just above 20 deg.
TILT_LIMITS = tuple(tuple(p) for p in ROBOT['safety']['tilt_limits'])   # (payload kg, limit deg), linear between
TILT_SLOW_BAND_DEG = ROBOT['safety']['tilt_slow_band_deg']   # within this of the limit: creep speed only
TILT_SLOW_SPEED = ROBOT['safety']['tilt_slow_speed']         # m/s - braking at 0.25 m/s on a slope tipped it
TILT_RELEASE_DEG = ROBOT['safety']['tilt_release_deg']       # hysteresis: a tilt stop releases below limit - this


NO_GO_LOOKAHEAD_M = ROBOT['safety']['no_go_lookahead_m']


def _inside(x, y, box):
    cx, cy, hx, hy = box
    return abs(x - cx) <= hx and abs(y - cy) <= hy


def _depth(x, y, box):
    """How far (x, y) is inside the box (<= 0: outside)."""
    cx, cy, hx, hy = box
    return min(hx - abs(x - cx), hy - abs(y - cy))


def tilt_limit(payload_kg):
    (p0, t0), (p1, t1) = TILT_LIMITS
    if payload_kg <= p0:
        return t0
    if payload_kg >= p1:
        return t1
    return t0 + (t1 - t0) * (payload_kg - p0) / (p1 - p0)


def tilt_from_quaternion(x, y, z, w):
    """Angle between the robot's up axis and vertical, degrees (pitch and
    roll combined; heading does not matter)."""
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


class SafetySupervisorCore:

    def __init__(self, require_operator_link=True, use_imu=False, payload_kg=0.0, zones=None):
        """zones: {name: (cx, cy, hx, hy)} no-go zones that apply at this
        payload (site_config.zones_blocking); needs the pose (on_pose)."""
        self.require_operator_link = require_operator_link
        self.zones = dict(zones or {})
        self.pose = None             # (x, y, yaw) in the map frame
        self.pose_t = None
        self.zone_hit = None         # name of the zone motion was blocked by
        self.use_imu = use_imu
        self.tilt_limit = tilt_limit(payload_kg)
        self.tilt = 0.0              # degrees, from the IMU
        self.imu_t = None
        self.tilt_stopped = False    # latched at the limit, released with hysteresis
        self.tilt_override = False   # operator 'resume' while tilt-stopped: creep allowed
        self.estop = False
        self.hold = False            # after clear_estop: zero until 'resume'
        self.cancelled = False       # after cancel: zero until a new goal
        self.cmd = (0.0, 0.0)
        self.cmd_t = None            # (wall, sim) receive times
        self.scan_t = None
        self.odom_t = None
        self.link_t = None           # wall only: mission_server runs on wall time
        self.front_clear = math.inf  # free distance ahead of the body front
        self.rear_clear = math.inf

    # -- inputs ------------------------------------------------------------
    # `sim_now` defaults to `now` (wall time), i.e. a real robot.
    def on_nav_cmd(self, linear, angular, now, sim_now=None):
        self.cmd = (float(linear), float(angular))
        self.cmd_t = (now, now if sim_now is None else sim_now)

    def on_mission_cmd(self, cmd):
        if cmd == 'estop':
            self.estop, self.hold = True, False
        elif cmd == 'clear_estop':
            if self.estop:
                self.estop, self.hold = False, True
        elif cmd == 'resume':
            self.hold = False
            if self.tilt_stopped:
                self.tilt_override = True
        elif cmd == 'cancel':
            self.cancelled = True

    def on_goal(self):
        self.cancelled = False

    def on_odom(self, now, sim_now=None):
        self.odom_t = (now, now if sim_now is None else sim_now)

    def on_imu(self, qx, qy, qz, qw, now, sim_now=None):
        self.imu_t = (now, now if sim_now is None else sim_now)
        self.tilt = tilt_from_quaternion(qx, qy, qz, qw)
        if self.tilt >= self.tilt_limit:
            self.tilt_stopped = True
        elif self.tilt < self.tilt_limit - TILT_RELEASE_DEG:
            self.tilt_stopped = self.tilt_override = False

    def on_pose(self, x, y, yaw, now, sim_now=None):
        self.pose = (x, y, yaw)
        self.pose_t = (now, now if sim_now is None else sim_now)

    def _zone_ahead(self, direction):
        """Name of a no-go zone the body would enter moving forward (+1) or
        backward (-1): probes across the leading edge, out to the lookahead."""
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        edge = BODY_FRONT_M if direction > 0 else -BODY_REAR_M
        for along in (0.0, NO_GO_LOOKAHEAD_M / 2, NO_GO_LOOKAHEAD_M):
            px = edge + direction * along
            for py in (-BODY_HALF_WIDTH_M, 0.0, BODY_HALF_WIDTH_M):
                wx, wy = x + c * px - s * py, y + s * px + c * py
                for name, box in self.zones.items():
                    if _inside(wx, wy, box):
                        return name
        return None

    def on_heartbeat(self, now):
        self.link_t = now

    def on_scan(self, ranges, angle_min, angle_increment, now, sim_now=None):
        """Raw laser-frame ranges. Updates the free distance in front of /
        behind the robot body within its width (self-occlusion masked)."""
        self.scan_t = (now, now if sim_now is None else sim_now)
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(r.size) * angle_increment
        deg = np.degrees(a)
        ok = np.isfinite(r) & (r >= LIDAR_MIN_RANGE_M)
        for lo, hi in SELF_BANDS_DEG:
            ok &= ~((deg >= lo) & (deg <= hi))
        x = LIDAR_X_OFFSET + r[ok] * np.cos(a[ok])     # base_link frame
        y = r[ok] * np.sin(a[ok])
        lane = np.abs(y) <= BODY_HALF_WIDTH_M
        ahead = x[lane & (x > BODY_FRONT_M)] - BODY_FRONT_M
        behind = -x[lane & (x < -BODY_REAR_M)] - BODY_REAR_M
        self.front_clear = float(ahead.min()) if ahead.size else math.inf
        self.rear_clear = float(behind.min()) if behind.size else math.inf

    # -- decision ----------------------------------------------------------
    @staticmethod
    def _stale(t, now, sim_now, timeout):
        return t is None or sim_now - t[1] > timeout or now - t[0] > WALL_BACKSTOP_S

    def output(self, now, sim_now=None):
        """Returns (linear, angular, reason, clamped). reason 'ok' = pass-through."""
        sim_now = now if sim_now is None else sim_now
        if self.estop:
            return 0.0, 0.0, 'estop', False
        if self.hold:
            return 0.0, 0.0, 'hold_after_estop', False
        if self.cancelled:
            return 0.0, 0.0, 'cancelled', False
        if self._stale(self.scan_t, now, sim_now, SENSOR_TIMEOUT_S):
            return 0.0, 0.0, 'stale_scan', False
        if self._stale(self.odom_t, now, sim_now, SENSOR_TIMEOUT_S):
            return 0.0, 0.0, 'stale_odom', False
        if self.use_imu and self._stale(self.imu_t, now, sim_now, SENSOR_TIMEOUT_S):
            return 0.0, 0.0, 'stale_imu', False
        if self.zones and self._stale(self.pose_t, now, sim_now, SENSOR_TIMEOUT_S):
            return 0.0, 0.0, 'stale_pose', False
        if self.require_operator_link and (self.link_t is None or now - self.link_t > LINK_TIMEOUT_S):
            return 0.0, 0.0, 'link_lost', False
        if self.use_imu and self.tilt_stopped and not self.tilt_override:
            return 0.0, 0.0, 'tilt_limit', False
        if self._stale(self.cmd_t, now, sim_now, CMD_TIMEOUT_S):
            return 0.0, 0.0, 'stale_cmd', False

        lin, ang = self.cmd
        max_lin = MAX_LINEAR
        self.zone_hit = None
        inside = [n for n, b in self.zones.items() if _inside(self.pose[0], self.pose[1], b)] if self.zones else []
        if inside:
            # already inside (e.g. placed there): only creep, and only in the
            # direction that takes the robot out (depth into the zone shrinks)
            max_lin = min(max_lin, TILT_SLOW_SPEED)
            if lin != 0.0:
                x, y, yaw = self.pose
                d = NO_GO_LOOKAHEAD_M * (1 if lin > 0 else -1)
                box = self.zones[inside[0]]
                if _depth(x + d * math.cos(yaw), y + d * math.sin(yaw), box) >= _depth(x, y, box):
                    self.zone_hit = inside[0]
                    return 0.0, max(-MAX_ANGULAR, min(MAX_ANGULAR, ang)), 'no_go_zone', False
        if self.use_imu and (self.tilt_override or self.tilt >= self.tilt_limit - TILT_SLOW_BAND_DEG):
            max_lin = TILT_SLOW_SPEED
        clamped = abs(lin) > max_lin or abs(ang) > MAX_ANGULAR
        lin = max(-max_lin, min(max_lin, lin))
        ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))
        if (lin > 0 and self.front_clear < GUARD_MARGIN_M) or \
                (lin < 0 and self.rear_clear < GUARD_MARGIN_M):
            return 0.0, ang, 'collision_guard', clamped
        if self.zones and lin != 0.0 and not inside:
            self.zone_hit = self._zone_ahead(1 if lin > 0 else -1)
            if self.zone_hit:
                return 0.0, ang, 'no_go_zone', clamped
        return lin, ang, 'ok', clamped
