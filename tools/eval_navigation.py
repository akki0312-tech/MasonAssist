#!/usr/bin/env python3
"""MasonAssist navigation evaluation.

Runs scenario sets repeatedly, each from a clean simulation restart with the
robot at the dock, and writes per-run metrics, traces, a summary and a path
plot. Uses the same HTTP API as the dashboard, so it tests the real stack.

  python3 tools/eval_navigation.py                       # default set (~45 min headless)
  python3 tools/eval_navigation.py --scenarios bricks --reps 1
  python3 tools/eval_navigation.py --gui                 # with the Gazebo window (slower)

Results: eval_results/<timestamp>/{runs.csv, summary.md, paths.png, traces/}
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request

WS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(WS, 'src', 'masonassist_description', 'scripts'))
from eval_metrics import (  # noqa: E402
    arrival_errors, body_clearance, path_length, planned_length, summarize,
)
from site_planner import SitePlanner  # noqa: E402
from transport_mission import MISSIONS, STATIONS  # noqa: E402
from worker_mover import OBSTACLES  # noqa: E402

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

API = 'http://localhost:8080'
SAMPLE_S = 0.2
RUN_TIMEOUT_S = 600
GOAL_TIMEOUT_S = 240                 # per map-click goal in the 'goals' scenario
SELF_BANDS_DEG = (96.8, 105.8)      # robot's own tracks, ignored for min LiDAR range
STILL_MPS = 0.01                     # odom vs ground truth compared only when stopped
DEBRIS_XY = (-3.17, -2.9)            # on the dock -> brick_pickup route; a detour exists
DEBRIS_SIZE = (1.0, 0.8, 0.6)
DOCK = STATIONS['dock'][:2]
GOALS = [(-5.5, -1.0), (-6.5, 0.5), (-3.7, 4.5), (4.5, -1.5), DOCK]
CUSTOM = ({'x': 4.5, 'y': -1.5}, {'x': -5.5, 'y': -1.0})

SCENARIOS = {   # name: (default reps, description)
    'bricks': (3, 'fixed job bricks_to_building'),
    'blocks': (3, 'fixed job blocks_to_scaffold'),
    'custom': (2, 'custom job, map point (4.5, -1.5) -> map point (-5.5, -1.0)'),
    'goals': (1, '5 chained map-click goals across the site'),
    'debris': (2, f'bricks_to_building with an unmapped {DEBRIS_SIZE[0]} x {DEBRIS_SIZE[1]} m box '
                  f'spawned on the route at {DEBRIS_XY}'),
}
METRICS = ['wall_s', 'sim_s', 'rtf', 'travelled_m', 'planned_m', 'path_ratio', 'min_body_clearance_m',
           'min_lidar_m', 'min_unmapped_m', 'slow_s', 'stop_s', 'replans', 'reroutes',
           'max_learned_cells', 'supervisor_stops', 'max_arrival_err_m', 'final_dock_err_m',
           'max_est_err_m', 'mean_est_err_m', 'final_est_err_m', 'max_odom_drift_m', 'gt_check_dev_m']
COLUMNS = ['scenario', 'rep', 'result', 'reason'] + METRICS + ['supervisor_reasons']


def call(path, body=None, timeout=5, attempts=3):
    """HTTP API call. Error replies (4xx) come back as their JSON; connection
    problems and timeouts are retried, then raised."""
    req = urllib.request.Request(API + path, json.dumps(body).encode() if body is not None else None,
                                 {'Content-Type': 'application/json'})
    for attempt in range(attempts):
        try:
            return json.loads(urllib.request.urlopen(req, timeout=timeout).read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())
        except (urllib.error.URLError, OSError):
            if attempt == attempts - 1:
                raise
            time.sleep(1.0)


def ground_truth():
    out = subprocess.run(['gz', 'model', '-m', 'masonassist', '-p'],
                         capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'XYZ.*?\n\s*\[?\s*([-\d.e]+)\s+([-\d.e]+)', out)
    return (float(m.group(1)), float(m.group(2))) if m else None


def gz_service(service, reqtype, req):
    return subprocess.run(['gz', 'service', '-s', service, '--reqtype', reqtype, '--reptype',
                           'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                          capture_output=True, text=True, timeout=10).stdout.strip()


def spawn_debris():
    sx, sy, sz = DEBRIS_SIZE
    geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    sdf = (f"<sdf version='1.9'><model name='eval_debris'><static>true</static>"
           f"<pose>{DEBRIS_XY[0]} {DEBRIS_XY[1]} {sz / 2} 0 0 0</pose><link name='l'>"
           f"<collision name='c'>{geom}</collision><visual name='v'>{geom}<material>"
           f"<ambient>0.55 0.35 0.15 1</ambient><diffuse>0.55 0.35 0.15 1</diffuse></material>"
           f"</visual></link></model></sdf>")
    return gz_service('/world/masonassist_world/create', 'gz.msgs.EntityFactory',
                      'sdf: "%s"' % sdf.replace('"', '\\"'))


class Recorder:
    """Latest values from ROS topics plus a sampled trace with run counters."""

    def __init__(self, node):
        self.pose = None          # TRUE pose (x, y, yaw) from /ground_truth/odom - all metrics use it
        self.belief = None        # the robot's own pose estimate (/robot_pose)
        self.odom = None          # raw odometry (/odom), to measure its drift
        self.v = 0.0
        self.sim = None
        self.safety = {}
        self.sup = None
        self.lidar_min = math.inf
        self.reset()
        node.create_subscription(Odometry, '/ground_truth/odom', self._truth, 20)
        node.create_subscription(Odometry, '/robot_pose', self._belief, 20)
        node.create_subscription(Odometry, '/odom', self._odom, 20)
        node.create_subscription(Clock, '/clock', self._clock, 10)
        node.create_subscription(String, '/safety_status', self._safety, 10)
        node.create_subscription(String, '/supervisor_status', self._sup, 20)
        node.create_subscription(LaserScan, '/scan', self._scan, 10)

    def reset(self):
        self.trace = []
        self.sup_events = {}
        self.lidar_run_min = math.inf
        self.unmapped_min = math.inf
        self.slow_s = self.stop_s = 0.0
        self.gt_dev = 0.0

    @staticmethod
    def _xyyaw(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        return (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _truth(self, m):
        self.pose = self._xyyaw(m)
        self.v = m.twist.twist.linear.x

    def _belief(self, m):
        self.belief = self._xyyaw(m)

    def _odom(self, m):
        self.odom = self._xyyaw(m)

    def _clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _safety(self, m):
        self.safety = json.loads(m.data)
        if self.safety.get('nearest_m') is not None:
            self.unmapped_min = min(self.unmapped_min, self.safety['nearest_m'])

    def _sup(self, m):
        reason = json.loads(m.data)['reason']
        if reason != self.sup and reason != 'ok':
            self.sup_events[reason] = self.sup_events.get(reason, 0) + 1
        self.sup = reason

    def _scan(self, m):
        best = math.inf
        for i, r in enumerate(m.ranges):
            deg = abs(math.degrees(m.angle_min + i * m.angle_increment))
            if math.isfinite(r) and not SELF_BANDS_DEG[0] <= deg <= SELF_BANDS_DEG[1]:
                best = min(best, r)
        self.lidar_min = best
        self.lidar_run_min = min(self.lidar_run_min, best)

    def sample(self, t, state):
        m = state.get('mission') or {}
        zone = self.safety.get('zone')
        if zone == 'slow':
            self.slow_s += SAMPLE_S
        elif zone == 'stop':
            self.stop_s += SAMPLE_S
        x, y, yaw = self.pose
        bx, by, _ = self.belief or (math.nan, math.nan, 0.0)
        ox, oy, _ = self.odom or (math.nan, math.nan, 0.0)
        self.trace.append({'t': round(t, 2), 'sim': self.sim, 'x': x, 'y': y, 'yaw': yaw, 'v': self.v,
                           'est_x': bx, 'est_y': by, 'est_err': math.hypot(bx - x, by - y),
                           'odom_err': math.hypot(ox - x, oy - y),
                           'phase': m.get('phase'), 'leg_index': m.get('leg_index'),
                           'payload': m.get('payload'), 'nav': state.get('mission_status'),
                           'zone': zone, 'sup': self.sup, 'cells': state['live_map']['count'],
                           'replans': m.get('replans'), 'reroutes': m.get('reroutes')})

    def check_ground_truth(self):
        """Sanity check of /ground_truth/odom against Gazebo's model pose
        (gz CLI), while the robot is still."""
        if self.pose is None or abs(self.v) > STILL_MPS:
            return
        before = self.pose
        gt = ground_truth()
        if gt and abs(self.v) <= STILL_MPS and math.hypot(self.pose[0] - before[0], self.pose[1] - before[1]) < 0.005:
            self.gt_dev = max(self.gt_dev, math.hypot(gt[0] - before[0], gt[1] - before[1]))


def restart(gui, odometry, localization):
    args = [os.path.join(WS, 'start_demo.sh'), '--restart', f'--odometry={odometry}',
            f'--localization={localization}'] + ([] if gui else ['--headless'])
    r = subprocess.run(args, capture_output=True, text=True, timeout=300)
    return r.returncode == 0, r.stdout.strip().splitlines()[-3:]


def wait_until(rec, done, timeout, t0, every=None):
    """Sample every SAMPLE_S until done(state) or timeout; returns last state."""
    last_gt = time.monotonic()
    while True:
        now = time.monotonic()
        state = call('/api/state')
        if rec.pose is not None:
            rec.sample(now - t0, state)
        if every:
            every(state)
        if done(state):
            return state, False
        if now - t0 > timeout:
            return state, True
        if now - last_gt > 5.0:
            rec.check_ground_truth()
            last_gt = time.monotonic()
        time.sleep(max(0.0, SAMPLE_S - (time.monotonic() - now)))


def run_job(rec, body, t0, during=None):
    """Queue one job and wait for it to land in history."""
    r = call('/api/queue', body)
    if 'error' in r:
        return 'rejected', r['error'], None
    state, timed_out = wait_until(rec, lambda s: bool(s['queue']['history']), RUN_TIMEOUT_S, t0, during)
    if timed_out:
        call('/api/cmd', {'cmd': 'cancel'})
        return 'timeout', f'no result after {RUN_TIMEOUT_S} s', state
    job = state['queue']['history'][0]
    return job['status'], job['reason'], state


def legs_targets(legs):
    return [(leg['x'], leg['y']) for leg in legs]


def run_scenario(name, rec, planner):
    t0 = time.monotonic()
    sim0 = rec.sim
    targets, result, reason = [], None, None
    if name in ('bricks', 'blocks', 'debris'):
        mission = 'blocks_to_scaffold' if name == 'blocks' else 'bricks_to_building'
        targets = [STATIONS[s][:2] for s, _ in MISSIONS[mission]['legs']]
        spawned = []

        def during(state):   # debris appears once the robot has driven 1 m
            if name == 'debris' and not spawned and rec.pose and math.hypot(*rec.pose[:2]) > 1.0:
                spawned.append(spawn_debris())
        result, reason, _ = run_job(rec, {'action': 'add', 'mission': mission}, t0, during)
        if name == 'debris':
            gz_service('/world/masonassist_world/remove', 'gz.msgs.Entity', 'name: "eval_debris" type: MODEL')
            if spawned != ['data: true']:
                result, reason = 'error', f'debris spawn failed: {spawned}'
    elif name == 'custom':
        pick, drop = CUSTOM
        targets = [(pick['x'], pick['y']), (drop['x'], drop['y']), DOCK]
        result, reason, _ = run_job(rec, {'action': 'add', 'mission': 'custom', 'pickup': pick,
                                          'dropoff': drop}, t0)
    elif name == 'goals':
        targets = list(GOALS)
        result = 'complete'
        for gx, gy in GOALS:
            r = call('/api/goal', {'x': gx, 'y': gy})
            if 'error' in r:
                result, reason = 'rejected', r['error']
                break
            state, timed_out = wait_until(
                rec, lambda s: (s['mission'] or {}).get('phase') in ('complete', 'failed', 'aborted'),
                time.monotonic() - t0 + GOAL_TIMEOUT_S, t0)
            phase = (state['mission'] or {}).get('phase')
            if timed_out or phase != 'complete':
                result = 'timeout' if timed_out else phase
                reason = (state['mission'] or {}).get('abort_reason') or f'goal ({gx}, {gy})'
                break
    wall = time.monotonic() - t0
    time.sleep(1.5)                                   # settle, then final ground truth
    gt = ground_truth()
    sim = (rec.sim - sim0) if (rec.sim is not None and sim0 is not None) else None
    trace = rec.trace
    xy = [(s['x'], s['y']) for s in trace]
    travelled = path_length(xy)
    planned = planned_length(DOCK, targets, planner) if targets else None
    errs = arrival_errors(trace, targets) if name != 'goals' else _goal_arrivals(trace, targets)
    row = {
        'result': result, 'reason': reason,
        'wall_s': round(wall, 1), 'sim_s': None if sim is None else round(sim, 1),
        'rtf': None if not sim else round(sim / wall, 2),
        'travelled_m': round(travelled, 2), 'planned_m': None if planned is None else round(planned, 2),
        'path_ratio': round(travelled / planned, 3) if planned else None,
        'min_body_clearance_m': round(min(body_clearance(s['x'], s['y'], s['yaw']) for s in trace), 3)
        if trace else None,
        'min_lidar_m': None if rec.lidar_run_min == math.inf else round(rec.lidar_run_min, 2),
        'min_unmapped_m': None if rec.unmapped_min == math.inf else round(rec.unmapped_min, 2),
        'slow_s': round(rec.slow_s, 1), 'stop_s': round(rec.stop_s, 1),
        'replans': max((s['replans'] or 0) for s in trace) if trace else None,
        'reroutes': max((s['reroutes'] or 0) for s in trace) if trace else None,
        'max_learned_cells': max(s['cells'] for s in trace) if trace else None,
        'supervisor_stops': sum(rec.sup_events.values()),
        'supervisor_reasons': ' '.join(f'{k}:{v}' for k, v in sorted(rec.sup_events.items())),
        'max_arrival_err_m': round(max(errs), 3) if errs else None,
        'final_dock_err_m': round(math.hypot(gt[0] - DOCK[0], gt[1] - DOCK[1]), 3) if gt else None,
        'max_est_err_m': round(max(s['est_err'] for s in trace), 3) if trace else None,
        'mean_est_err_m': round(sum(s['est_err'] for s in trace) / len(trace), 3) if trace else None,
        'final_est_err_m': round(trace[-1]['est_err'], 3) if trace else None,
        'max_odom_drift_m': round(max(s['odom_err'] for s in trace), 3) if trace else None,
        'gt_check_dev_m': round(max(rec.gt_dev, math.hypot(gt[0] - rec.pose[0], gt[1] - rec.pose[1])), 3)
        if gt and rec.pose else None,
    }
    return row


def _goal_arrivals(trace, goals):
    """Goals: one leg each, so arrival = each switch into 'complete'."""
    errs, prev, k = [], None, 0
    for s in trace:
        if s['phase'] == 'complete' and prev != 'complete' and k < len(goals):
            errs.append(math.hypot(s['x'] - goals[k][0], s['y'] - goals[k][1]))
            k += 1
        prev = s['phase']
    return errs


def fmt(st, unit='', nd=1):
    if st is None:
        return '-'
    if st['min'] == st['max']:
        return f"{st['mean']:.{nd}f}{unit}"
    return f"{st['mean']:.{nd}f} ± {st['sd']:.{nd}f}{unit} ({st['min']:.{nd}f}–{st['max']:.{nd}f})"


def write_summary(out, rows, started, gui, config):
    s = summarize(rows, METRICS)
    lines = [f'# Navigation evaluation - {started}', '',
             f"{len(rows)} runs, {config}, {'Gazebo GUI' if gui else 'headless'}, each from a clean restart with the robot "
             'at the dock. Mean ± sd (min–max) over successful runs.', '',
             '| Scenario | Success | Time (s) | Travelled (m) | Path ratio | Min body clearance (m) | '
             'Max arrival error (m) | Replans / reroutes | Learned cells (max) | Supervisor stops | '
             'Pose estimate error, max (m) | Raw odometry drift, max (m) |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for name, v in s.items():
        st = v['stats']
        ok = [r for r in rows if r['scenario'] == name and r['result'] == 'complete']
        mn = lambda k: min((r[k] for r in ok if r[k] is not None), default=None)   # noqa: E731
        mx = lambda k: max((r[k] for r in ok if r[k] is not None), default=None)   # noqa: E731
        lines.append(
            f"| {name} | {v['ok']}/{v['runs']} | {fmt(st['wall_s'])} | {fmt(st['travelled_m'], nd=2)} | "
            f"{fmt(st['path_ratio'], nd=2)} | {mn('min_body_clearance_m')} | {mx('max_arrival_err_m')} | "
            f"{sum(r['replans'] or 0 for r in ok)} / {sum(r['reroutes'] or 0 for r in ok)} | "
            f"{mx('max_learned_cells')} | {sum(r['supervisor_stops'] or 0 for r in ok)} | "
            f"{mx('max_est_err_m')} | {mx('max_odom_drift_m')} |")
    failures = [r for r in rows if r['result'] != 'complete']
    lines += ['', '**Scenarios**', ''] + [f'- `{k}`: {SCENARIOS[k][1]}' for k in s]
    lines += ['', '**Failures**', ''] + ([f"- {r['scenario']} #{r['rep']}: {r['result']} - {r['reason']}"
                                          for r in failures] or ['- none'])
    lines += ['', 'Path ratio = distance driven ÷ shortest planned route on the known layout (1.00 = the '
              'planned route; slightly below 1 is normal because the robot cuts waypoint corners within '
              '0.35 m and stops within 0.35 m of each station; above 1 = detours). '
              'Body clearance = smallest gap between the robot footprint and any known obstacle box '
              "(the boxes cover each object's collision shape, so this is conservative). "
              'Supervisor stops = times the safety supervisor overrode navigation. Pose estimate error = '
              'largest gap between where the robot thought it was and where it really was (Gazebo ground '
              'truth), sampled every 0.2 s through the run. All other metrics use the ground truth.']
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return '\n'.join(lines)


def plot_paths(out, rows, traces):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(8, 8))
    for cx, cy, hx, hy in OBSTACLES.values():
        ax.add_patch(Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy, color='#9aa4b5'))
    ax.add_patch(Rectangle((DEBRIS_XY[0] - DEBRIS_SIZE[0] / 2, DEBRIS_XY[1] - DEBRIS_SIZE[1] / 2),
                           DEBRIS_SIZE[0], DEBRIS_SIZE[1], fill=False, ls='--', ec='#b05a1a',
                           label='debris (debris scenario only)'))
    for n, (x, y, _) in STATIONS.items():
        ax.plot(x, y, 's', color='#2f5d8a')
        ax.annotate(n, (x, y), textcoords='offset points', xytext=(6, 4), fontsize=8)
    colours = dict(zip(SCENARIOS, ['#1f77b4', '#2ca02c', '#9467bd', '#ff7f0e', '#d62728']))
    shown = set()
    for r, tr in zip(rows, traces):
        if not tr:
            continue
        ax.plot([s['x'] for s in tr], [s['y'] for s in tr], color=colours[r['scenario']], lw=1, alpha=0.7,
                label=None if r['scenario'] in shown else r['scenario'])
        shown.add(r['scenario'])
    ax.set_xlim(-8.2, 8.2)
    ax.set_ylim(-8.2, 8.2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Driven paths (odometry) on the known site layout')
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'paths.png'), dpi=110)


class _Tee:
    """Console output also goes to <out>/log.txt."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for st in self.streams:
            st.write(text)
            st.flush()

    def flush(self):
        for st in self.streams:
            st.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenarios', default=','.join(SCENARIOS), help='comma list: ' + ','.join(SCENARIOS))
    ap.add_argument('--reps', type=int, help='repetitions per scenario (default: per-scenario)')
    ap.add_argument('--gui', action='store_true', help='run with the Gazebo window')
    ap.add_argument('--odometry', choices=['perfect', 'drift'], default='drift',
                    help='odometry the robot uses (start_demo.sh --odometry)')
    ap.add_argument('--localization', choices=['none', 'amcl'], default='amcl',
                    help='localisation (start_demo.sh --localization)')
    ap.add_argument('--out', help='output directory (default eval_results/<timestamp>)')
    args = ap.parse_args()
    names = [n.strip() for n in args.scenarios.split(',') if n.strip()]
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        ap.error(f'unknown scenario(s): {unknown}')
    started = time.strftime('%Y-%m-%d %H:%M')
    out = args.out or os.path.join(WS, 'eval_results', time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(out, 'traces'), exist_ok=True)
    sys.stdout = _Tee(sys.stdout, open(os.path.join(out, 'log.txt'), 'w'))

    rclpy.init()
    node = rclpy.create_node('eval_navigation')
    rec = Recorder(node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    planner = SitePlanner()

    rows, traces = [], []
    plan = [(n, k + 1) for n in names for k in range(args.reps or SCENARIOS[n][0])]
    for i, (name, rep) in enumerate(plan, 1):
        print(f'[{i}/{len(plan)}] {name} #{rep}: restarting simulation...', flush=True)
        ok, tail = restart(args.gui, args.odometry, args.localization)
        rec.reset()
        t_wait = time.monotonic()
        while ok and (rec.pose is None or rec.sim is None) and time.monotonic() - t_wait < 30:
            time.sleep(0.2)
        if not ok or rec.pose is None:
            row = {c: None for c in COLUMNS}
            row.update(scenario=name, rep=rep, result='start_failed', reason=' / '.join(tail))
            traces.append([])
        else:
            try:
                row = run_scenario(name, rec, planner)
            except Exception:   # a tool/stack problem must not end the batch mid-start
                print(traceback.format_exc(), flush=True)
                call_ok = False
                try:
                    call('/api/cmd', {'cmd': 'cancel'}, attempts=1)
                    call_ok = True
                except Exception:
                    pass
                row = {c: None for c in COLUMNS}
                row.update(result='error', reason=traceback.format_exc().strip().splitlines()[-1]
                           + ('' if call_ok else ' (API unreachable)'))
            row.update(scenario=name, rep=rep)
            traces.append(rec.trace)
            with open(os.path.join(out, 'traces', f'{name}_{rep}.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rec.trace[0].keys()) if rec.trace else ['t'])
                w.writeheader()
                w.writerows(rec.trace)
        rows.append(row)
        print(f"    {row['result']} in {row['wall_s']} s, travelled {row['travelled_m']} m, path ratio "
              f"{row['path_ratio']}, min clearance {row['min_body_clearance_m']} m, dock error "
              f"{row['final_dock_err_m']} m" + (f" - {row['reason']}" if row['reason'] else ''), flush=True)
        with open(os.path.join(out, 'runs.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS)
            w.writeheader()
            w.writerows(rows)

    print('\n' + write_summary(out, rows, started, args.gui, f'odometry={args.odometry}, localization={args.localization}'))
    plot_paths(out, rows, traces)
    print(f'\nResults: {out}')
    os._exit(0)


if __name__ == '__main__':
    main()
