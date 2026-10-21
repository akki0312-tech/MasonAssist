#!/usr/bin/env python3
"""Mission dashboard backend: a single process combining an rclpy node and
a tiny dependency-free HTTP server (Python stdlib http.server only - no
Flask/rosbridge, neither of which is installed in this environment).

Subscribes to real topics (/odom, /scan, /mission_status, /cmd_vel) and
exposes their current values over a small JSON API. Publishes user actions
(/goal_pose, /mission_cmd) that vfh_plus_node.py already consumes. This
node never computes navigation itself and never fabricates sensor data -
every field in the API response is either a real subscribed value or
explicitly null/absent if that topic has not published yet.
"""
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Reuse worker_mover.py's own OBSTACLES registry as the single source of
# truth for the dashboard's "known site layout" overlay, instead of
# keeping a second, driftable copy of the same coordinates.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worker_mover import OBSTACLES  # noqa: E402
from transport_mission import MISSIONS, STATIONS, TransportMission, validate_goal  # noqa: E402
from site_planner import SitePlanner  # noqa: E402
from job_queue import JobQueue  # noqa: E402

HTTP_PORT = 8080
DASHBOARD_HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dashboard.html')


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionServerNode(Node):
    def __init__(self):
        super().__init__('mission_server')
        self.lock = threading.Lock()
        self.state = {
            'odom': None,            # {x,y,yaw} or None if never received
            'odom_twist': None,      # {lin_x, ang_z} - odometry's OWN estimated
                                     # velocity (dead-reckoning), distinct from
                                     # the commanded /cmd_vel below
            'scan': None,            # {n,finite,min_finite,angle_min,angle_increment,
                                     #  ranges} - full scan so the dashboard can
                                     # plot real LiDAR points, or None
            'cmd_vel': None,         # {lin_x, ang_z} - COMMANDED velocity or None
            'mission_status': 'unknown',
            'goal': None,            # {x,y} or None - last goal this server sent
            'safety': None,          # {zone,nearest_m,unmapped} from vfh_plus_node, or None
        }

        self.goal_pub = self.create_publisher(Point, '/goal_pose', 10)
        self.cmd_pub = self.create_publisher(String, '/mission_cmd', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(String, '/mission_status', self._status_cb, 10)
        self.create_subscription(String, '/safety_status', self._safety_cb, 10)

        self.mission = None  # TransportMission or None
        self.queue = JobQueue()
        self.job_mission = None  # the TransportMission running queue.current, if any
        self.planner = SitePlanner()
        self.create_timer(0.2, self._mission_tick)

        self.get_logger().info(f'mission_server started, dashboard at http://0.0.0.0:{HTTP_PORT}/')

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        t = msg.twist.twist
        with self.lock:
            self.state['odom'] = {'x': p.x, 'y': p.y, 'yaw': _yaw_of(msg.pose.pose.orientation)}
            self.state['odom_twist'] = {'lin_x': t.linear.x, 'ang_z': t.angular.z}

    def _scan_cb(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r)]
        with self.lock:
            self.state['scan'] = {
                'n': len(msg.ranges),
                'finite': len(finite),
                'min_finite': min(finite) if finite else None,
                'angle_min': msg.angle_min,
                'angle_increment': msg.angle_increment,
                # Full range array so the dashboard can plot real LiDAR
                # returns in world frame (robot pose + per-beam angle).
                # inf/nan left as-is; the client filters them, same as
                # the finite-count logic above.
                'ranges': [r if math.isfinite(r) else None for r in msg.ranges],
            }

    def _cmd_vel_cb(self, msg: Twist) -> None:
        with self.lock:
            self.state['cmd_vel'] = {'lin_x': msg.linear.x, 'ang_z': msg.angular.z}

    def _status_cb(self, msg: String) -> None:
        with self.lock:
            self.state['mission_status'] = msg.data

    def _safety_cb(self, msg: String) -> None:
        try:
            safety = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.state['safety'] = safety

    def _now(self) -> float:
        return time.monotonic()

    def _mission_tick(self) -> None:
        now = self._now()
        goal = None
        just_failed = False
        with self.lock:
            if self.mission is not None:
                odom = self.state['odom']
                odom_xy = (odom['x'], odom['y']) if odom else None
                was_active = self.mission.active
                goal = self.mission.update(now, odom_xy, self.state['mission_status'])
                just_failed = was_active and self.mission.phase == 'failed'
            self._record_finished_job(now)
            next_goal = self._start_next_job(now)
        if goal is not None:
            self.publish_goal(*goal)
        if next_goal is not None:
            self.publish_goal(*next_goal)
        if just_failed:
            # stop VFH+ driving toward the abandoned goal (same path as operator cancel)
            self.get_logger().warn(f'mission failed: {self.mission.abort_reason}')
            self.cmd_pub.publish(String(data='cancel'))
            with self.lock:
                self.state['goal'] = None

    def _record_finished_job(self, now) -> None:
        """Call with self.lock held. Moves the running job to history once
        its mission is over (complete / failed / aborted - including aborts
        caused by cancel or a manual map goal)."""
        job, m = self.queue.current, self.job_mission
        if job is None or (m is not None and m.active):
            return
        status = m.phase if m is not None and m.phase in ('complete', 'failed') else 'aborted'
        reason = m.abort_reason if m is not None else 'mission lost'
        self.queue.finish(job, status, reason, m.replans if m is not None else 0, now)
        self.job_mission = None
        self.get_logger().info(f'job #{job.id} {job.mission}: {status}' + (f' ({reason})' if reason else ''))

    def _start_next_job(self, now):
        """Call with self.lock held. Starts the next queued job if the robot
        is free. Returns the first goal to publish, or None."""
        if self.mission is not None and self.mission.active:
            return None
        job = self.queue.next_to_start()
        if job is None:
            return None
        odom = self.state['odom']
        start = (odom['x'], odom['y']) if odom else None
        m = TransportMission(job.mission, now, start, self.planner)
        self.queue.mark_started(job, now)
        self.mission = m
        self.job_mission = m
        goal = m.current_goal()
        if goal is None:
            self.queue.finish(job, 'failed', m.abort_reason, 0, now)
            self.job_mission = None
            return None
        self.get_logger().info(f'job #{job.id} started: {job.mission}')
        return goal

    def queue_action(self, data):
        """POST /api/queue. Returns (http_code, message)."""
        action = data.get('action')
        now = self._now()
        with self.lock:
            try:
                if action == 'add':
                    job = self.queue.add(str(data.get('mission', '')), now)
                    return 200, f'job #{job.id} queued'
                if action == 'remove':
                    self.queue.remove(int(data.get('id', -1)))
                    return 200, 'removed'
                if action == 'clear':
                    self.queue.clear()
                    return 200, 'cleared'
                if action == 'pause':
                    self.queue.pause()
                    return 200, 'queue paused'
                if action == 'resume':
                    self.queue.resume()
                    return 200, 'queue resumed'
            except (ValueError, TypeError) as ex:
                return 400, str(ex)
        return 400, 'expected action: add|remove|clear|pause|resume'

    def get_state_json(self) -> bytes:
        with self.lock:
            now = self._now()
            snapshot = dict(self.state)
            snapshot['mission'] = self.mission.to_dict(now) if self.mission else None
            snapshot['queue'] = self.queue.to_dict(now)
        return json.dumps(snapshot).encode('utf-8')

    def publish_goal(self, x: float, y: float) -> None:
        self.goal_pub.publish(Point(x=float(x), y=float(y), z=0.0))
        with self.lock:
            self.state['goal'] = {'x': float(x), 'y': float(y)}

    def publish_cmd(self, cmd: str) -> None:
        self.cmd_pub.publish(String(data=cmd))
        if cmd == 'cancel':
            with self.lock:
                self.state['goal'] = None
                if self.mission is not None:
                    self.mission.abort('cancelled', self._now())

    def manual_goal(self, x: float, y: float):
        """Operator-picked goal: rejects points outside the site or inside a
        known obstacle, otherwise aborts any active mission and drives a
        planned 'goto' mission there (same A* routing as transport
        missions - raw goals straight to VFH+ hit the same corner traps).
        Returns (http_code, message)."""
        error = validate_goal(x, y)
        if error:
            return 400, error
        with self.lock:
            if self.mission is not None:
                self.mission.abort('manual goal override', self._now())
            odom = self.state['odom']
            start = (odom['x'], odom['y']) if odom else None
            self.mission = TransportMission.goto(x, y, self._now(), start, self.planner)
            goal = self.mission.current_goal()
            if goal is None:
                return 409, f'cannot drive there: {self.mission.abort_reason}'
        self.publish_goal(*goal)
        return 200, 'ok'

    def start_mission(self, name: str):
        """Returns (http_code, message)."""
        if name not in MISSIONS:
            return 400, f'unknown mission {name!r}'
        with self.lock:
            if self.mission is not None and self.mission.active:
                return 409, f'mission {self.mission.name!r} already active - cancel it first'
            odom = self.state['odom']
            start = (odom['x'], odom['y']) if odom else None
            self.mission = TransportMission(name, self._now(), start, self.planner)
            goal = self.mission.current_goal()
            if goal is None:
                return 409, f'cannot start: {self.mission.abort_reason}'
        self.publish_goal(*goal)
        self.get_logger().info(f'transport mission started: {name}')
        return 200, 'ok'


def make_handler(node: MissionServerNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # keep the terminal quiet during the demo

        def end_headers(self):
            # without this Firefox kept showing a stale cached dashboard.html
            # after updates (observed: old page with no mission/safety panels)
            self.send_header('Cache-Control', 'no-store')
            super().end_headers()

        def _send_json(self, obj, code=200):
            body = json.dumps(obj).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/' or self.path == '/index.html':
                try:
                    with open(DASHBOARD_HTML_PATH, 'rb') as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except FileNotFoundError:
                    self._send_json({'error': 'dashboard.html not found'}, 500)
            elif self.path == '/api/state':
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                body = node.get_state_json()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/api/layout':
                # The REAL, authored site geometry - the exact same
                # OBSTACLES dict worker_mover.py validates worker paths
                # against (imported, not duplicated). This is the known
                # site layout, not a planner-derived or fabricated map.
                layout = [
                    {'name': name, 'cx': cx, 'cy': cy, 'hx': hx, 'hy': hy}
                    for name, (cx, cy, hx, hy) in OBSTACLES.items()
                ]
                self._send_json({'obstacles': layout})
            elif self.path == '/api/stations':
                self._send_json({
                    'stations': [{'name': n, 'x': x, 'y': y, 'kind': k}
                                 for n, (x, y, k) in STATIONS.items()],
                    'missions': [{'name': n, 'material': s['material'],
                                  'legs': [{'station': st, 'action': a} for st, a in s['legs']]}
                                 for n, s in MISSIONS.items()],
                })
            else:
                self._send_json({'error': 'not found'}, 404)

        def do_POST(self):
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length) if length else b'{}'
            try:
                data = json.loads(raw.decode('utf-8') or '{}')
            except json.JSONDecodeError:
                self._send_json({'error': 'bad json'}, 400)
                return

            if self.path == '/api/goal':
                try:
                    x, y = float(data['x']), float(data['y'])
                except (KeyError, TypeError, ValueError):
                    self._send_json({'error': 'expected {"x":..,"y":..}'}, 400)
                    return
                code, message = node.manual_goal(x, y)
                if code == 200:
                    self._send_json({'ok': True})
                else:
                    self._send_json({'error': message}, code)
            elif self.path == '/api/queue':
                code, message = node.queue_action(data)
                if code == 200:
                    self._send_json({'ok': True, 'message': message})
                else:
                    self._send_json({'error': message}, code)
            elif self.path == '/api/mission':
                code, message = node.start_mission(str(data.get('mission', '')))
                if code == 200:
                    self._send_json({'ok': True})
                else:
                    self._send_json({'error': message}, code)
            elif self.path == '/api/cmd':
                cmd = data.get('cmd')
                if cmd not in ('cancel', 'pause', 'resume', 'estop', 'clear_estop'):
                    self._send_json({'error': 'bad cmd'}, 400)
                    return
                node.publish_cmd(cmd)
                self._send_json({'ok': True})
            else:
                self._send_json({'error': 'not found'}, 404)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = MissionServerNode()

    server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), make_handler(node))
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
