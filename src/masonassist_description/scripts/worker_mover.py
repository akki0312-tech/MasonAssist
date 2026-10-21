#!/usr/bin/env python3
"""Moves the construction_site.world 'worker' models along a looping
waypoint path, by calling Gazebo's own /world/<world>/set_pose service
(gz-sim-user-commands-system, already loaded by every world in this
project) at a fixed rate. This is real, continuous simulated motion -
the workers are ordinary collidable models being teleported each tick,
not a pre-rendered animation and not a robot-perception shortcut.

WHY NOT GAZEBO ACTORS (checked twice now, not assumed):
1. SDF <actor> was tried first and empirically confirmed (live test,
   SceneManager.cc warnings, worker invisible, absent from the model
   list) to not render primitive <visual> geometry in this installed
   Gazebo Harmonic version without a real skin mesh file.
2. Separately, and regardless of (1): per Gazebo's own actor docs
   (https://gazebosim.org/docs/harmonic/actors/), scripted actor
   trajectories do not participate in ordinary physics collisions at
   all - an actor's visual mesh can pass through walls even with
   collision geometry attached, because the trajectory is applied
   directly to the pose, bypassing the physics/contact pipeline the
   same way this script's own set_pose calls do.
So actors would not have solved the wall-crossing defect either way;
the real fix is choosing waypoints that are geometrically verified
clear, which is what this file now does explicitly (see
OBSTACLES/_clear_of_obstacles below), rather than hoping a path "looks"
clear.

PREVIOUS DEFECT (now fixed): worker_2's old lane ran along x=6 for
y in [-6,6], which passed directly through partial_wall_1 at (6,5.5) -
confirmed analytically, not by inspection, before this fix. Both lanes
below were chosen and then validated against every obstacle in
construction_site.world with a 0.3m margin before being written into
that file, and are re-validated here at every startup as a live check,
not just a one-time design note.
"""
import math
import sys
import time

WORLD = 'masonassist_world'
TICK_S = 0.1

# Every static obstacle in construction_site.world, as an axis-aligned
# bounding box (expanded conservatively for rotated boxes). This is the
# SAME list used to design the lanes below - kept here, not just in a
# design note, so a future change to either file can be re-checked by
# running this script (it validates and reports before moving anything).
# Format: name -> (center_x, center_y, half_x, half_y)
OBSTACLES = {
    'wall_north': (0, 8, 8.0, 0.2),
    'wall_south': (0, -8, 8.0, 0.2),
    'wall_east': (8, 0, 0.2, 8.0),
    'wall_west_a': (-8, 4.5, 0.2, 3.6),
    'wall_west_b': (-8, -4.5, 0.2, 3.6),
    'column_ne': (7.85, 7.85, 0.2, 0.2),
    'column_nw': (-7.85, 7.85, 0.2, 0.2),
    'column_se': (7.85, -7.85, 0.2, 0.2),
    'gate_post_a': (-8, 1.3, 0.2, 0.2),
    'gate_post_b': (-8, -1.3, 0.2, 0.2),
    'wall_a_left': (3.65, 2.7, 0.7, 0.2),
    'wall_a_right': (6.35, 2.7, 0.7, 0.2),
    'wall_b_lower': (3.0, 3.7, 0.2, 0.9),
    'wall_b_upper': (3.0, 6.0, 0.2, 1.2),
    'col_bld_1': (7.0, 2.7, 0.25, 0.25),
    'col_bld_2': (3.0, 7.3, 0.25, 0.25),
    'pallet_bricks': (-6, -7, 0.75, 0.65),
    'pallet_blocks': (-3, -7, 0.75, 0.65),
    'cement_bags': (0, -7, 0.75, 0.65),
    'pipe_pile': (3, -7, 0.75, 0.65),
    'timber_stack': (6, -7, 0.75, 0.65),
    'rebar_bundle': (-7.2, -7, 0.5, 0.5),
    'partition_a': (0, 1.1, 1.6, 0.25),
    'partition_b': (0, -1.1, 1.6, 0.25),
    'barrier_1': (2, -3, 0.75, 0.75),
    'safety_zone': (-1, 3.8, 0.3, 1.0),
    'rubble_1': (-2, 3, 0.3, 0.35),
    'rubble_2': (-1.6, 3.3, 0.25, 0.25),
    'scaffold_1': (2.2, 3.5, 0.4, 0.7),
    'warning_sign': (4.0, 1.6, 0.35, 0.1),
    'sand_pile': (-4.5, -7, 0.65, 0.65),
    'aggregate_pile': (1.5, -7, 0.6, 0.6),
}
ROBOT_SPAWN = (0.0, 0.0)

# The worker's own body extent, taken directly from its collision box in
# construction_site.world (<collision><geometry><box><size>0.5 0.35 1.8),
# i.e. half_x=0.25, half_y=0.175 - not an arbitrary guess. This is what
# "check their complete bodies, not just their centre points" means in
# this check: the clearance test below adds this extent to each
# obstacle's own half-extent (a Minkowski-sum style check), so a hit is
# reported only when the worker's actual body footprint would overlap
# the obstacle, not just its center point.
WORKER_HALF_X_M = 0.25
WORKER_HALF_Y_M = 0.175
# Independent safety buffer ON TOP of the real body extent above.
SAFETY_BUFFER_M = 0.15

# NOTE on orientation: both lanes below only ever use yaw 0 or pi (never
# a 90-degree turn), so the worker's local x/y collision-box extent stays
# aligned with world x/y throughout - this check does not rotate the
# worker's footprint, and would need to if a future lane turned sideways.

# Each entry: (model_name, waypoints) where waypoints is a list of
# (time_s, x, y, yaw) and the path loops back to the first waypoint.
# worker_1: open south lane, between Zone C and the Zone B storage strip.
# worker_2: open north lane, between the perimeter wall and Zone A.
WORKERS = {
    'worker_1': [
        (0.0, -6.5, -5.5, 0.0),
        (7.0, 6.3, -5.5, 0.0),
        (9.0, 6.3, -5.5, math.pi),
        (16.0, -6.5, -5.5, math.pi),
        (18.0, -6.5, -5.5, 0.0),
    ],
    'worker_2': [
        (0.0, -6.5, 7.0, 0.0),
        (6.0, 1.2, 7.0, 0.0),
        (8.0, 1.2, 7.0, math.pi),
        (14.0, -6.5, 7.0, math.pi),
        (16.0, -6.5, 7.0, 0.0),
    ],
}
# The human figure's link-local offsets (helmet/head/torso/legs) build
# upward from the link origin, so the teleported pose must sit at
# ground level (z=0), not at a pre-offset mid-height.
WORKER_HEIGHT = 0.0

# Walking-gait cue (whole-body bob + lean, NOT per-limb swing - see
# module docstring in the previous iteration's notes for why per-limb
# animation was rejected: no joint-position service exists, and
# teleporting 4+ extra limb entities per worker per tick would add
# enough subprocess overhead to risk the simulation's own performance).
WALK_FREQ_HZ = 1.8
BOB_AMPLITUDE_M = 0.025
LEAN_AMPLITUDE_RAD = 0.07


def _clear_of_obstacles(x, y):
    """Returns a list of obstacle names whose box, EXPANDED by the
    worker's own real body half-extent plus SAFETY_BUFFER_M, contains
    (x, y). This checks the worker's actual footprint against each
    obstacle's actual footprint (Minkowski sum), not a point-vs-point
    distance with an arbitrary flat margin."""
    hits = []
    for name, (cx, cy, hx, hy) in OBSTACLES.items():
        if (abs(x - cx) < hx + WORKER_HALF_X_M + SAFETY_BUFFER_M and
                abs(y - cy) < hy + WORKER_HALF_Y_M + SAFETY_BUFFER_M):
            hits.append(name)
    return hits


def _validate_path(name, waypoints, samples_per_segment=20):
    """Walks the full waypoint loop at fine resolution and checks every
    sampled point against every obstacle AND the robot's spawn point.
    This is the concrete implementation of 'before moving a worker,
    check whether the proposed movement crosses an obstacle' - it runs
    once at startup, before the first set_pose call, and refuses to
    move a worker whose path fails."""
    problems = []
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(waypoints, waypoints[1:]):
        for i in range(samples_per_segment + 1):
            frac = i / samples_per_segment
            x = x0 + frac * (x1 - x0)
            y = y0 + frac * (y1 - y0)
            hits = _clear_of_obstacles(x, y)
            if hits:
                problems.append((round(x, 2), round(y, 2), hits))
            d_spawn = math.hypot(x - ROBOT_SPAWN[0], y - ROBOT_SPAWN[1])
            if d_spawn < 0.8:
                problems.append((round(x, 2), round(y, 2), ['robot_spawn']))
    if problems:
        print(f'worker_mover: REFUSING to move {name} - path crosses an '
              f'obstacle or the robot spawn point at: {problems[:5]}'
              f'{" ..." if len(problems) > 5 else ""}', file=sys.stderr, flush=True)
        return False
    print(f'worker_mover: {name} path validated clear '
          f'(body half-extent {WORKER_HALF_X_M}x{WORKER_HALF_Y_M}m + '
          f'{SAFETY_BUFFER_M}m buffer, {len(OBSTACLES)} obstacles checked)', flush=True)
    return True


def _interp(waypoints, t):
    period = waypoints[-1][0]
    t = t % period
    for (t0, x0, y0, yaw0), (t1, x1, y1, yaw1) in zip(waypoints, waypoints[1:]):
        if t0 <= t <= t1:
            frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            x = x0 + frac * (x1 - x0)
            y = y0 + frac * (y1 - y0)
            dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
            yaw = yaw0 + frac * dyaw
            return x, y, yaw
    return waypoints[0][1], waypoints[0][2], waypoints[0][3]


def _yaw_pitch_quaternion(yaw, pitch):
    """Combined yaw(Z)-then-pitch(Y) quaternion (Hamilton product
    q_yaw * q_pitch), so the pitch lean is expressed in the body's own
    forward direction after yaw, not the world X axis."""
    hy, hp = yaw / 2.0, pitch / 2.0
    cy, sy = math.cos(hy), math.sin(hy)
    cp, sp = math.cos(hp), math.sin(hp)
    w = cy * cp
    x = -sy * sp
    y = cy * sp
    z = sy * cp
    return x, y, z, w


SET_POSE_SERVICE = f'/world/{WORLD}/set_pose'   # bridged by gazebo.launch.py


def _pose_request(name, x, y, z, yaw, pitch=0.0):
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    qx, qy, qz, qw = _yaw_pitch_quaternion(yaw, pitch)
    req = SetEntityPose.Request()
    req.entity.name = name
    req.entity.type = Entity.MODEL
    p = req.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return req


def main():
    valid_workers = {}
    for name, waypoints in WORKERS.items():
        if _validate_path(name, waypoints):
            valid_workers[name] = waypoints
    if not valid_workers:
        print('worker_mover: no worker paths passed validation - exiting without '
              'moving anything.', file=sys.stderr, flush=True)
        sys.exit(1)

    # One persistent ROS client to the bridged Gazebo service. The previous
    # version launched a `gz service` CLI process per worker per tick: each
    # took ~0.25 s and created a new transport node, so the loop really ran
    # at ~2 Hz and dragged the simulation to ~0.66x real time (measured:
    # 0.99x with this process paused).
    import rclpy
    from ros_gz_interfaces.srv import SetEntityPose
    rclpy.init()
    node = rclpy.create_node('worker_mover')
    client = node.create_client(SetEntityPose, SET_POSE_SERVICE)
    if not client.wait_for_service(timeout_sec=30.0):
        print(f'worker_mover: service {SET_POSE_SERVICE} not available - is the '
              'simulation (gazebo.launch.py) running?', file=sys.stderr, flush=True)
        sys.exit(1)

    pending = {}  # worker -> future; at most one request in flight per worker
    start = time.monotonic()
    print('worker_mover started - moving', list(valid_workers.keys()), flush=True)
    while rclpy.ok():
        tick_start = time.monotonic()
        t = tick_start - start
        walk_phase = 2.0 * math.pi * WALK_FREQ_HZ * t
        bob = BOB_AMPLITUDE_M * abs(math.sin(walk_phase))
        lean = LEAN_AMPLITUDE_RAD * math.sin(walk_phase)
        for name, waypoints in valid_workers.items():
            if name in pending and not pending[name].done():
                continue
            x, y, yaw = _interp(waypoints, t)
            pending[name] = client.call_async(
                _pose_request(name, x, y, WORKER_HEIGHT + bob, yaw, pitch=lean))
        while time.monotonic() - tick_start < TICK_S:
            rclpy.spin_once(node, timeout_sec=max(0.0, TICK_S - (time.monotonic() - tick_start)))


if __name__ == '__main__':
    main()
