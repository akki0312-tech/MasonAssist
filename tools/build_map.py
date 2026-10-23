#!/usr/bin/env python3
"""Build the site map with SLAM.

Starts the stack with drifting odometry, slam_toolbox and the workers
moved outside the site (so people are not baked into the map), drives a
coverage tour of the site
through the HTTP API, saves the map, and checks it against the known site
layout.

  python3 tools/build_map.py                 # -> src/masonassist_description/maps/construction_site.{yaml,pgm}
  python3 tools/build_map.py --out /tmp/test_map

Then rebuild (colcon build --symlink-install) so the map is installed, and
start with ./start_demo.sh --odometry=drift --localization=amcl.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

WS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(WS, 'src', 'masonassist_description', 'scripts'))
from eval_metrics import map_quality  # noqa: E402
from worker_mover import OBSTACLES  # noqa: E402

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

API = 'http://localhost:8080'
DEFAULT_OUT = os.path.join(WS, 'src', 'masonassist_description', 'maps', 'construction_site')
# Checked offline against the planner: every goal is reachable and clear (~76 m).
TOUR = [(-2.5, 0.0), (-5.5, -1.0), (-6.5, 0.5), (-4.0, 1.0), (-3.7, 4.5), (-1.6, 5.6), (1.0, 4.6),
        (-0.5, 6.6), (-3.0, 6.6), (-2.5, 0.0), (2.0, 0.4), (5.2, 1.25), (4.5, -1.5), (3.9, -4.8),
        (0.8, -4.9), (-3.0, -4.5), (-6.0, -4.5), (-2.5, -2.0), (0.0, 0.0)]
GOAL_TIMEOUT_S = 240


def call(path, body=None):
    req = urllib.request.Request(API + path, json.dumps(body).encode() if body is not None else None,
                                 {'Content-Type': 'application/json'})
    try:
        return json.loads(urllib.request.urlopen(req, timeout=3).read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


class Poses:
    """Latest true pose, raw odometry and SLAM pose, plus their errors."""

    def __init__(self, node):
        self.p = {}
        self.err = {'odom': [], 'slam': []}
        for key, topic in (('truth', '/ground_truth/odom'), ('odom', '/odom'), ('slam', '/robot_pose')):
            node.create_subscription(Odometry, topic, lambda m, k=key: self._cb(k, m), 20)

    def _cb(self, key, m):
        self.p[key] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def sample(self):
        t = self.p.get('truth')
        if t is None:
            return
        for k in ('odom', 'slam'):
            if k in self.p:
                self.err[k].append(math.hypot(self.p[k][0] - t[0], self.p[k][1] - t[1]))


def read_map(prefix):
    """Occupied cell centres (site frame) from a map_saver yaml + pgm."""
    meta = {}
    for line in open(prefix + '.yaml'):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    with open(prefix + '.pgm', 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = [int(v) for v in line.split()]
        int(f.readline())
        data = f.read()
    occupied = []
    for row in range(h):
        for col in range(w):
            if data[row * w + col] == 0:            # map_saver trinary: 0 = occupied
                occupied.append((ox + (col + 0.5) * res, oy + (h - row - 0.5) * res))
    return occupied, (w, h, res, ox, oy), data


def plot_map(prefix, size, data, png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Rectangle
    w, h, res, ox, oy = size
    img = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, cmap='gray', extent=(ox, ox + w * res, oy, oy + h * res), vmin=0, vmax=255)
    for cx, cy, hx, hy in OBSTACLES.values():
        ax.add_patch(Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy, fill=False, ec='#e0a030', lw=0.8))
    ax.set_xlim(-8.5, 8.5)
    ax.set_ylim(-8.5, 8.5)
    ax.set_title('SLAM map (black = occupied) with the known layout boxes (orange)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(png, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=DEFAULT_OUT, help='map path prefix (writes .yaml and .pgm)')
    ap.add_argument('--png', default=os.path.join(WS, 'docs', 'map', 'construction_site_map.png'))
    args = ap.parse_args()

    print('Starting the stack: drifting odometry + slam_toolbox, no workers...', flush=True)
    r = subprocess.run([os.path.join(WS, 'start_demo.sh'), '--restart', '--headless', '--odometry=drift',
                        '--localization=slam', '--no-workers'], capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        sys.exit('start failed:\n' + r.stdout + r.stderr)
    print(r.stdout.strip().splitlines()[-3], flush=True)

    # Without worker_mover the workers stand still at their start points and
    # would be mapped as walls: move them outside the site for the tour.
    for i, name in enumerate(('worker_1', 'worker_2')):
        out = subprocess.run(['gz', 'service', '-s', '/world/masonassist_world/set_pose', '--reqtype',
                              'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
                              f'name: "{name}", position: {{x: 30, y: {5 * i}, z: 0}}'],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        if out != 'data: true':
            sys.exit(f'could not move {name} out of the site: {out!r}')
    print('Workers moved outside the site for mapping', flush=True)

    rclpy.init()
    node = rclpy.create_node('build_map')
    poses = Poses(node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(3.0)

    t0 = time.monotonic()
    for i, (x, y) in enumerate(TOUR, 1):
        res = call('/api/goal', {'x': x, 'y': y})
        if 'error' in res:
            sys.exit(f'goal {i} ({x}, {y}) rejected: {res["error"]}')
        tg = time.monotonic()
        while True:
            poses.sample()
            m = call('/api/state')['mission'] or {}
            if m.get('phase') in ('complete', 'failed', 'aborted') or time.monotonic() - tg > GOAL_TIMEOUT_S:
                break
            time.sleep(0.5)
        print(f'  goal {i:2d}/{len(TOUR)} ({x:5.1f}, {y:5.1f}): {m.get("phase")} in {time.monotonic() - tg:5.1f} s'
              + (f' - {m.get("abort_reason")}' if m.get('abort_reason') else ''), flush=True)
    print(f'Tour done in {time.monotonic() - t0:.0f} s', flush=True)
    for k, label in (('odom', 'raw odometry'), ('slam', 'SLAM pose')):
        e = poses.err[k]
        if e:
            print(f'  {label} vs ground truth: mean {sum(e) / len(e):.3f} m, max {max(e):.3f} m, '
                  f'at the end {e[-1]:.3f} m', flush=True)

    time.sleep(3.0)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', args.out,
                        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'],
                       capture_output=True, text=True, timeout=60)
    if r.returncode != 0 or not os.path.exists(args.out + '.yaml'):
        sys.exit('map_saver failed:\n' + r.stdout + r.stderr)
    print(f'Map saved: {args.out}.yaml / .pgm', flush=True)

    occupied, size, data = read_map(args.out)
    q = map_quality(occupied)
    print(f"Map check: {q['occupied_cells']} occupied cells; {q['precision']:.1%} lie on known objects "
          f"(within 0.15 m); {q['recall']:.1%} of known object outlines were mapped", flush=True)
    missed = sorted((v, k) for k, v in q['per_box'].items() if v < 0.2)
    if missed:
        print('  objects mostly not in the map (lower than the LiDAR plane, or never seen): '
              + ', '.join(f'{k} {v:.0%}' for v, k in missed), flush=True)
    os.makedirs(os.path.dirname(args.png), exist_ok=True)
    plot_map(args.out, size, data, args.png)
    print(f'Map image: {args.png}', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
