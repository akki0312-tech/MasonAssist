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
MAX_LINEAR = 0.25            # m/s
MAX_ANGULAR = 0.5            # rad/s

# collision guard: robot body in base_link (chassis 0.725 x 0.525 m, tracks
# out to ~0.43 m each side), plus a stopping margin in the direction of travel
BODY_FRONT_M = 0.3625
BODY_REAR_M = 0.3625
BODY_HALF_WIDTH_M = 0.43
GUARD_MARGIN_M = 0.15
LIDAR_X_OFFSET = 0.30
LIDAR_MIN_RANGE_M = 0.1      # sensor <min>; returns inside the body
                             # rectangle are excluded by the lane test below
SELF_BANDS_DEG = ((-105.8, -96.8), (96.8, 105.8))   # same as vfh_plus_node


class SafetySupervisorCore:

    def __init__(self, require_operator_link=True):
        self.require_operator_link = require_operator_link
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
        elif cmd == 'cancel':
            self.cancelled = True

    def on_goal(self):
        self.cancelled = False

    def on_odom(self, now, sim_now=None):
        self.odom_t = (now, now if sim_now is None else sim_now)

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
        if self.require_operator_link and (self.link_t is None or now - self.link_t > LINK_TIMEOUT_S):
            return 0.0, 0.0, 'link_lost', False
        if self._stale(self.cmd_t, now, sim_now, CMD_TIMEOUT_S):
            return 0.0, 0.0, 'stale_cmd', False

        lin, ang = self.cmd
        clamped = abs(lin) > MAX_LINEAR or abs(ang) > MAX_ANGULAR
        lin = max(-MAX_LINEAR, min(MAX_LINEAR, lin))
        ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))
        if (lin > 0 and self.front_clear < GUARD_MARGIN_M) or \
                (lin < 0 and self.rear_clear < GUARD_MARGIN_M):
            return 0.0, ang, 'collision_guard', clamped
        return lin, ang, 'ok', clamped
