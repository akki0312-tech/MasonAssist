#!/usr/bin/env python3
"""MasonAssist payload & terrain test matrix (simulation only).

For each payload, starts launch/terrain_test.launch.py (terrain world, the
robot with that payload, no navigation or supervisor) and drives the robot
directly on each lane of worlds/terrain_test.world:

  climb   each ramp lane (slope x surface, tools/make_terrain_world.py) at
          0.2 m/s: reached the plateau?, time, slip (1 - actual/commanded
          speed on the slope), max pitch/roll
  hill    the same ramp at 0.25 m/s with a stop half way: stopping distance,
          slide-back over 5 s, then a hill start to the plateau
  down    down the ramp at 0.25 m/s, zero command half way: stopping
          distance and slide over 5 s
  flat    0.25 m/s on the level, zero command: stopping distance
  cross   along a 10 deg side slope at 0.2 m/s: max roll, sideways drift

  python3 tools/terrain_test.py                                   # 0/15/25/50 kg, ~80 min
  python3 tools/terrain_test.py --payloads 0 --lanes concrete_10  # quick check
  python3 tools/terrain_test.py --supervisor                      # tilt-guard checks, through
                                                                  # the safety supervisor

Gazebo's track model (surface-motion conveyor, mu 0.9) is a rough stand-in
for real tracks on real ground: treat the numbers as relative, not absolute.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

WS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from make_terrain_world import (  # noqa: E402
    CROSS_SLOPE_DEG as CROSS_DEG, LANES as LANE_SPEC, PLATEAU, RAMP_LEN, SURFACES, X0, cross_slope_y, lane_y,
)
# lane key 'surface_deg' -> (surface, slope deg, lane centre y)
LANES = {f'{s}_{d}': (s, d, lane_y(i)) for i, (s, d) in enumerate(LANE_SPEC)}
CROSS_Y = cross_slope_y() + 1.5 * math.cos(math.radians(CROSS_DEG))
TIP_MARGIN_DEG = 10.0                              # pitch/roll this far beyond the ground's own tilt = unstable
STOPPED_MPS = 0.01
OUT_DEFAULT = os.path.join(WS, 'docs', 'terrain')
COLUMNS = ['payload_kg', 'test', 'lane', 'surface', 'mu', 'slope_deg', 'speed', 'result', 'time_s', 'slip', 'max_pitch_deg',
           'max_roll_deg', 'stop_dist_m', 'slide_m', 'side_drift_m_per_m', 'note']


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Rig:
    """Commands /cmd_vel, reads /ground_truth/odom, teleports via set_pose."""

    def __init__(self, node, cmd_topic='/cmd_vel'):
        self.node = node
        self.state = None            # dict: x y z roll pitch yaw v t
        self.cmd = None
        self.sup_seen = set()        # supervisor reasons since reset_sup()
        self.sup_tilt = 0.0          # largest tilt the supervisor reported
        self.pub = node.create_publisher(Twist, cmd_topic, 10)
        node.create_subscription(String, '/supervisor_status', self._sup, 20)
        node.create_subscription(Odometry, '/ground_truth/odom', self._odom, 20)
        self.set_pose = node.create_client(SetEntityPose, '/world/masonassist_world/set_pose')
        threading.Thread(target=self._cmd_loop, daemon=True).start()

    def _odom(self, m):
        p = m.pose.pose.position
        r, pi, y = rpy(m.pose.pose.orientation)
        v = m.twist.twist.linear
        self.state = {'x': p.x, 'y': p.y, 'z': p.z, 'roll': r, 'pitch': pi, 'yaw': y,
                      'v': math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) * (1 if v.x >= 0 else -1),
                      't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}

    def _sup(self, m):
        d = json.loads(m.data)
        self.sup_seen.add(d['reason'])
        if d.get('tilt_deg') is not None:
            self.sup_tilt = max(self.sup_tilt, d['tilt_deg'])

    def reset_sup(self):
        self.sup_seen, self.sup_tilt = set(), 0.0

    def _cmd_loop(self):
        while True:
            t = Twist()
            t.linear.x = self.cmd or 0.0
            self.pub.publish(t)
            time.sleep(0.05)

    def place(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, settle=2.5):
        self.cmd = 0.0
        req = SetEntityPose.Request()
        req.entity.name = 'masonassist'
        req.entity.type = Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        o = req.pose.orientation
        o.x, o.y, o.z, o.w = quat(roll, pitch, yaw)
        if not self.set_pose.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('set_pose service not available')
        fut = self.set_pose.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        if not fut.done() or not fut.result().success:
            raise RuntimeError('set_pose failed')
        time.sleep(settle)

    def run(self, speed, until, timeout):
        """Drive at `speed` until until(state) or timeout (sim time).
        Returns (samples, reached)."""
        samples, t0 = [], None
        self.cmd = speed
        while True:
            s = dict(self.state)
            samples.append(s)
            t0 = s['t'] if t0 is None else t0
            if until(s):
                return samples, True
            if s['t'] - t0 > timeout:
                return samples, False
            time.sleep(0.02)

    def hold(self, seconds):
        """Zero command for `seconds` of sim time; returns samples."""
        self.cmd = 0.0
        samples, t0 = [], self.state['t']
        while self.state['t'] - t0 < seconds:
            samples.append(dict(self.state))
            time.sleep(0.02)
        return samples


def dist(a, b):
    return math.sqrt((a['x'] - b['x']) ** 2 + (a['y'] - b['y']) ** 2 + (a['z'] - b['z']) ** 2)


def peak(samples, key):
    return round(max(abs(math.degrees(s[key])) for s in samples), 1)


def unstable(samples, lane_y, slope_deg=0.0, roll_deg=0.0):
    """'' if the robot stayed on the ground's own tilt and in its lane."""
    if any(abs(math.degrees(s['pitch'])) > slope_deg + TIP_MARGIN_DEG or
           abs(math.degrees(s['roll'])) > roll_deg + TIP_MARGIN_DEG for s in samples):
        return 'tip-over risk'
    if lane_y is not None and any(abs(s['y'] - lane_y) > 0.8 for s in samples):
        return 'left the lane'
    return ''


def stopping(rig, lane_y):
    """Zero the command; distance until stopped, then slide over 5 s."""
    start = dict(rig.state)
    samples = rig.hold(8.0)
    stop_i = next((i for i, s in enumerate(samples) if abs(s['v']) < STOPPED_MPS), len(samples) - 1)
    stopped = samples[stop_i]
    later = [s for s in samples if s['t'] >= stopped['t'] + 5.0] or samples[-1:]
    return round(dist(start, stopped), 3), round(dist(stopped, later[0]), 3), samples


def ramp_geometry(deg):
    th = math.radians(deg)
    return th, RAMP_LEN * math.cos(th), RAMP_LEN * math.sin(th)


def test_payload(rig, payload, lanes, rows, profiles):
    def row(**kw):
        r = {c: None for c in COLUMNS}
        r.update(payload_kg=payload, **kw)
        rows.append(r)
        print('   ', {k: v for k, v in r.items() if v not in (None, '')}, flush=True)

    # flat stopping distance
    rig.place(-6.0, 0.0, 0.1)
    rig.run(0.25, lambda s: False, 5.0)
    d, slide, _ = stopping(rig, 0.0)
    row(test='flat_stop', lane='flat', surface='ground', mu=1.0, slope_deg=0, speed=0.25, result='ok',
        stop_dist_m=d, slide_m=slide)

    for key in lanes:
        surface, deg, y = LANES[key]
        lane = dict(lane=key, surface=surface, mu=SURFACES[surface][0], slope_deg=deg)
        th, run, h = ramp_geometry(deg)
        top_x = X0 + run + 0.6

        # climb at 0.2 m/s, no stop
        rig.place(-1.5, y, 0.1)
        samples, ok = rig.run(0.2, lambda s: s['x'] > top_x, (run + 3.0) / 0.2 * 2.5)
        on_ramp = [s for s in samples if X0 + 0.5 < s['x'] < X0 + run - 0.5]
        slip = None
        if len(on_ramp) > 2:
            along = sum(dist(a, b) for a, b in zip(on_ramp, on_ramp[1:]))
            slip = round(1.0 - along / (on_ramp[-1]['t'] - on_ramp[0]['t']) / 0.2, 3)
        note = unstable(samples, y, deg)
        if not ok and not note and samples[-1]['x'] < X0 + 1.0:
            note = 'stalled at the level-to-slope corner'
        row(test='climb', **lane, speed=0.2, result='ok' if ok and not note else ('unstable' if 'tip' in note or 'lane' in note else 'failed'),
            time_s=round(samples[-1]['t'] - samples[0]['t'], 1), slip=slip, max_pitch_deg=peak(samples, 'pitch'),
            max_roll_deg=peak(samples, 'roll'), note=note)
        profiles.append((payload, key, [(s['x'], math.degrees(s['pitch'])) for s in samples]))

        # placed on the slope a quarter of the way up (no level-to-slope corner):
        # slide over 5 s with zero command, then a hill start to the plateau
        d0 = RAMP_LEN / 4
        rig.place(X0 + d0 * math.cos(th), y, d0 * math.sin(th) + 0.1, pitch=-th)
        held = rig.hold(5.0)
        slide = round(dist(held[0], held[-1]), 3)
        samples, ok = rig.run(0.25, lambda s: s['x'] > top_x, (run + 1.0) / 0.25 * 3)
        # skip the first second: the 0.5 m/s^2 drive limit is still accelerating
        on_ramp = [s for s in samples if s['x'] < X0 + run - 0.5 and s['t'] > samples[0]['t'] + 1.0]
        hs_slip = None
        if len(on_ramp) > 10 and on_ramp[-1]['t'] > on_ramp[0]['t'] + 1.0:
            along = sum(dist(a, b) for a, b in zip(on_ramp, on_ramp[1:]))
            hs_slip = round(1.0 - along / (on_ramp[-1]['t'] - on_ramp[0]['t']) / 0.25, 3)
        note = unstable(held + samples, y, deg)
        row(test='hold_hill_start', **lane, speed=0.25,
            result='ok' if ok and not note else ('unstable' if note else 'no hill start'),
            slide_m=slide, slip=hs_slip, max_pitch_deg=peak(held + samples, 'pitch'), note=note)

        # down the far ramp at 0.25 m/s, zero command half way down
        x1 = X0 + run + PLATEAU
        rig.place(x1 - 1.2, y, h + 0.1)
        rig.run(0.25, lambda s: s['x'] > x1 + run / 2, 60.0)
        d, slide, held = stopping(rig, y)
        note = unstable(held, y, deg)
        row(test='downhill_stop', **lane, speed=0.25, result='unstable' if note else 'ok',
            stop_dist_m=d, slide_m=slide, max_pitch_deg=peak(held, 'pitch'), note=note)

    # cross slope: drive along a 10 deg side tilt
    th = math.radians(CROSS_DEG)
    z = 1.5 * math.sin(th) + 0.12
    rig.place(-1.5, CROSS_Y, z, roll=th)
    y0 = rig.state['y']
    samples, _ = rig.run(0.2, lambda s: s['x'] > 5.5, 60.0)
    travelled = math.hypot(samples[-1]['x'] - samples[0]['x'], samples[-1]['y'] - samples[0]['y'])
    note = unstable(samples, None, 0.0, CROSS_DEG)
    row(test='cross_slope', lane=f'cross_{CROSS_DEG}', surface='concrete', mu=1.0, slope_deg=CROSS_DEG, speed=0.2, result='unstable' if note else 'ok',
        max_roll_deg=peak(samples, 'roll'), side_drift_m_per_m=round((y0 - samples[-1]['y']) / max(travelled, 0.1), 3),
        time_s=round(samples[-1]['t'] - samples[0]['t'], 1), note=note)


GUARD_CASES = {50.0: ['concrete_20', 'concrete_30', 'concrete_35', 'gravel_30'], 0.0: ['concrete_30']}


def guard_tests(rig, payload, rows):
    """Tilt guard + no-go zones (config/terrain_site.yaml) through the safety
    supervisor: the cases that tipped or slid (50 kg on 30/35 deg) and
    controls (50 kg on 20 deg, empty on 30 deg)."""
    def row(test, key, samples, reached, extra=''):
        surface, deg, y = LANES[key]
        tipped = rig.sup_tilt > deg + TIP_MARGIN_DEG or any(
            max(abs(math.degrees(s['pitch'])), abs(math.degrees(s['roll']))) > deg + TIP_MARGIN_DEG for s in samples)
        r = {c: None for c in COLUMNS}
        r.update(payload_kg=payload, test=test, lane=key, surface=surface, mu=SURFACES[surface][0],
                 slope_deg=deg, speed=0.25, max_pitch_deg=peak(samples, 'pitch'),
                 result='TIPPED' if tipped else ('reached' if reached else 'stopped'),
                 note=f"supervisor: {','.join(sorted(rig.sup_seen))}; max tilt {rig.sup_tilt:.1f}°" + extra)
        rows.append(r)
        print('   ', {k: v for k, v in r.items() if v not in (None, '')}, flush=True)

    for key in GUARD_CASES.get(payload, []):
        surface, deg, y = LANES[key]
        th, run, h = ramp_geometry(deg)
        x1 = X0 + run + PLATEAU
        # down the far ramp from the plateau (the case that tipped at 50 kg)
        rig.place(x1 - 1.2, y, h + 0.1)
        rig.reset_sup()
        samples, reached = rig.run(0.25, lambda s: s['x'] > x1 + run / 2, 40.0)
        held = rig.hold(5.0)
        front = samples[-1]['x'] + 0.3625
        row('guard_downhill', key, samples + held, reached,
            f"; stopped with its front {x1 - front:+.2f} m before the crest" if not reached else '')
        # placed on the slope with zero command (static case), then: try to
        # drive up (deeper into a no-go zone), then reverse back down and out
        d0 = RAMP_LEN / 4
        rig.place(X0 + d0 * math.cos(th), y, d0 * math.sin(th) + 0.1, pitch=-th)
        rig.reset_sup()
        held = rig.hold(4.0)
        x_start = rig.state['x']
        up, _ = rig.run(0.25, lambda s: s['x'] > x_start + 0.5, 6.0)
        moved_up = up[-1]['x'] - x_start
        out, left = rig.run(-0.25, lambda s: s['x'] < X0 - 0.1, 40.0)
        row('guard_placed', key, held + up + out, left,
            f"; drove up {moved_up:+.2f} m when asked; reversed off the ramp: {'yes' if left else 'no'}")
        # climb from level ground
        rig.place(-1.5, y, 0.1)
        rig.reset_sup()
        samples, reached = rig.run(0.25, lambda s: s['x'] > X0 + run + 0.6, 60.0)
        row('guard_climb', key, samples, reached,
            f"; front {X0 - (samples[-1]['x'] + 0.3625):+.2f} m before the ramp foot" if not reached else '')


def start_stack(payload, gui, supervisor=False):
    subprocess.run([os.path.join(WS, 'stop_demo.sh')], capture_output=True, timeout=60)
    log = open(f'/tmp/masonassist_terrain_{payload:g}kg.log', 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'masonassist_description', 'terrain_test.launch.py',
                             f'payload_kg:={payload:g}', f'headless:={"false" if gui else "true"}',
                             f'supervisor:={"true" if supervisor else "false"}'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc


def stop_stack(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except Exception:
        os.killpg(proc.pid, signal.SIGKILL)
    subprocess.run([os.path.join(WS, 'stop_demo.sh')], capture_output=True, timeout=60)


def write_outputs(out, rows, profiles):
    with open(os.path.join(out, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    payloads = sorted({r['payload_kg'] for r in rows})
    lanes = list(dict.fromkeys(r['lane'] for r in rows if r['test'] == 'climb'))

    def cell(test, p, lane, fmt):
        r = next((r for r in rows if r['test'] == test and r['payload_kg'] == p and r['lane'] == lane), None)
        return fmt(r) if r else '-'
    lines = ['# Payload & terrain test (simulation)', '',
             'Vehicle only: no navigation or safety supervisor; commands sent straight to the tracks. '
             'Robot 100 kg + payload; drive acceleration limited to 0.5 m/s². Gazebo track model with '
             'Coulomb friction (track mu 0.9, surface mu as listed) - read the numbers as relative, not absolute.',
             '']
    head = '| Payload | ' + ' | '.join(f"{LANES[k][1]}° {LANES[k][0]} (μ {SURFACES[LANES[k][0]][0]})" for k in lanes) + ' |'
    sep = '|---|' + '---|' * len(lanes)

    def climb_fmt(r):
        s = '✓' if r['result'] == 'ok' else f"✗ {r['note'] or r['result']}"
        return f"{s} {r['time_s']} s, slip {r['slip']:.0%}, pitch {r['max_pitch_deg']}°" if r['slip'] is not None \
            else f"{s} {r['time_s']} s"
    for title, test, fmt in [
            ('Climb from level ground at 0.2 m/s (time to plateau, slip on the slope, max pitch)', 'climb', climb_fmt),
            ('Placed on the slope: slide in 5 s with zero command / hill start at 0.25 m/s (slip)', 'hold_hill_start',
             lambda r: f"{r['slide_m']} m / " + (f"✓ slip {r['slip']:.0%}" if r['result'] == 'ok' and r['slip'] is not None
                                                  else ('✓' if r['result'] == 'ok' else '✗ ' + r['result']))),
            ('Downhill at 0.25 m/s, zero command half way down: stopping distance / slide in 5 s', 'downhill_stop',
             lambda r: f"{r['stop_dist_m']} m / {r['slide_m']} m" + ('' if r['result'] == 'ok' else f" ✗ {r['result']}"))]:
        lines += [f'**{title}**', '', head, sep]
        for p in payloads:
            lines.append(f'| {p:g} kg | ' + ' | '.join(cell(test, p, d, fmt) for d in lanes) + ' |')
        lines.append('')
    lines += ['**Level ground and side slope**', '',
              '| Payload | Flat stop from 0.25 m/s | 10° side slope: max roll / sideways drift per m |', '|---|---|---|']
    for p in payloads:
        lines.append(f"| {p:g} kg | {cell('flat_stop', p, 'flat', lambda r: str(r['stop_dist_m']) + ' m')} | "
                     f"{cell('cross_slope', p, f'cross_{CROSS_DEG}', lambda r: f'''{r['max_roll_deg']}° / {r['side_drift_m_per_m']} m''')} |")
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(payloads), figsize=(4.2 * len(payloads), 3.6), sharey=True, squeeze=False)
    for ax, p in zip(axes[0], payloads):
        for payload, key, prof in profiles:
            if payload == p:
                ax.plot([x for x, _ in prof], [pt for _, pt in prof], label=key.replace('_', ' ') + '°')
        ax.set_title(f'{p:g} kg payload: pitch while climbing')
        ax.set_xlabel('x (m)')
        ax.grid(alpha=0.3)
    axes[0][0].set_ylabel('pitch (deg, nose up < 0)')
    axes[0][0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'pitch_profiles.png'), dpi=100)
    return '\n'.join(lines)


def write_guard_outputs(out, rows):
    with open(os.path.join(out, 'supervisor_guard.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    lines = ['# Tilt guard through the safety supervisor (simulation)', '',
             'Same vehicle and terrain as summary.md, but commands go through the safety supervisor '
             'with its IMU tilt guard (limit 35° up to 25 kg, 20° at 50 kg; creep speed 0.1 m/s within 5° '
             'of the limit; stop at the limit). Commanded speed 0.25 m/s.', '',
             '| Payload | Lane | Test | Result | Max pitch | Supervisor |', '|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f"| {r['payload_kg']:g} kg | {r['lane']} | {r['test'].replace('guard_', '')} | {r['result']} | "
                     f"{r['max_pitch_deg']}° | {r['note']} |")
    with open(os.path.join(out, 'supervisor_guard.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--payloads', default='0,15,25,50', help='kg, comma list')
    ap.add_argument('--lanes', default=','.join(LANES), help='ramp lanes to test: ' + ','.join(LANES))
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--supervisor', action='store_true',
                    help='tilt-guard checks through the safety supervisor (writes supervisor_guard.*)')
    ap.add_argument('--out', default=OUT_DEFAULT)
    args = ap.parse_args()
    payloads = [float(p) for p in args.payloads.split(',')]
    lanes = [k.strip() for k in args.lanes.split(',') if k.strip()]
    unknown = [k for k in lanes if k not in LANES]
    if unknown:
        ap.error(f'unknown lane(s): {unknown}')
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('terrain_test', parameter_overrides=[])
    rig = Rig(node, '/cmd_vel_nav' if args.supervisor else '/cmd_vel')
    if args.supervisor and args.payloads == '0,15,25,50':
        payloads = sorted(GUARD_CASES, reverse=True)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    rows, profiles = [], []
    for payload in payloads:
        print(f'[{payload:g} kg] starting terrain world...', flush=True)
        proc = start_stack(payload, args.gui, args.supervisor)
        try:
            t0 = time.monotonic()
            rig.state = None
            while rig.state is None and time.monotonic() - t0 < 120:
                time.sleep(0.5)
            if rig.state is None:
                print(f'[{payload:g} kg] no /ground_truth/odom - see /tmp/masonassist_terrain_{payload:g}kg.log')
                continue
            time.sleep(3.0)
            if args.supervisor:
                guard_tests(rig, payload, rows)
            else:
                test_payload(rig, payload, lanes, rows, profiles)
        except Exception as ex:   # record and move on to the next payload
            r = {c: None for c in COLUMNS}
            r.update(payload_kg=payload, test='error', result='error', note=repr(ex))
            rows.append(r)
            print(f'[{payload:g} kg] error: {ex!r}', flush=True)
        finally:
            rig.cmd = 0.0
            stop_stack(proc)
    if args.supervisor:
        print('\n' + write_guard_outputs(args.out, rows))
    else:
        print('\n' + write_outputs(args.out, rows, profiles))
    print(f'\nResults: {args.out}')
    os._exit(0)


if __name__ == '__main__':
    main()
