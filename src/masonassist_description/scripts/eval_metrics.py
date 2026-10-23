#!/usr/bin/env python3
"""ROS-free metrics for the navigation evaluation (tools/eval_navigation.py).

A trace is a list of samples recorded during one run; each sample is a dict
with at least t, x, y, yaw and, where known, phase and leg_index.
"""

import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from safety_supervisor_core import BODY_FRONT_M, BODY_HALF_WIDTH_M, BODY_REAR_M  # noqa: E402
from worker_mover import OBSTACLES  # noqa: E402

JITTER_M = 0.01      # pose changes below this are noise, not travel
ARRIVAL_PHASES = ('loading', 'unloading', 'complete')


def path_length(poses, jitter=JITTER_M):
    """Distance travelled along (x, y) poses, ignoring sub-jitter wobble."""
    total = 0.0
    last = None
    for x, y in poses:
        if last is None:
            last = (x, y)
            continue
        d = math.hypot(x - last[0], y - last[1])
        if d >= jitter:
            total += d
            last = (x, y)
    return total


def route_length(start, route):
    pts = [tuple(start)] + [tuple(p) for p in route]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def planned_length(start, targets, planner):
    """Shortest planned length from start through each target in turn, or
    None if any leg has no route."""
    total, pos = 0.0, tuple(start)
    for target in targets:
        route = planner.plan(pos, tuple(target))
        if route is None:
            return None
        total += route_length(pos, route)
        pos = tuple(target)
    return total


def footprint(x, y, yaw):
    """Robot body corners in the world frame (counter-clockwise)."""
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(BODY_FRONT_M, BODY_HALF_WIDTH_M), (-BODY_REAR_M, BODY_HALF_WIDTH_M),
             (-BODY_REAR_M, -BODY_HALF_WIDTH_M), (BODY_FRONT_M, -BODY_HALF_WIDTH_M)]
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in local]


def _seg_point(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _overlap(poly_a, poly_b):
    """Separating-axis test for two convex polygons."""
    for poly in (poly_a, poly_b):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = y1 - y2, x2 - x1
            pa = [nx * px + ny * py for px, py in poly_a]
            pb = [nx * px + ny * py for px, py in poly_b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def polygon_distance(poly_a, poly_b):
    if _overlap(poly_a, poly_b):
        return 0.0
    best = math.inf
    for p, q in ((poly_a, poly_b), (poly_b, poly_a)):
        for pt in p:
            for i in range(len(q)):
                best = min(best, _seg_point(pt, q[i], q[(i + 1) % len(q)]))
    return best


def _box(cx, cy, hx, hy):
    return [(cx + hx, cy + hy), (cx - hx, cy + hy), (cx - hx, cy - hy), (cx + hx, cy - hy)]


def body_clearance(x, y, yaw, boxes=None):
    """Smallest gap between the robot body and any known obstacle box."""
    body = footprint(x, y, yaw)
    boxes = OBSTACLES.values() if boxes is None else boxes
    return min(polygon_distance(body, _box(*b)) for b in boxes)


def arrival_errors(trace, targets):
    """Distance to the leg target each time a station is reached: on entering
    loading / unloading / complete. targets: [(x, y)] per leg."""
    errors = []
    prev = None
    for s in trace:
        phase = s.get('phase')
        if phase in ARRIVAL_PHASES and phase != prev and targets:
            leg = min(s.get('leg_index') or 0, len(targets) - 1)
            tx, ty = targets[leg]
            errors.append(math.hypot(s['x'] - tx, s['y'] - ty))
        prev = phase
    return errors


def _stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {'mean': statistics.fmean(values),
            'sd': statistics.stdev(values) if len(values) > 1 else 0.0,
            'min': min(values), 'max': max(values)}


def summarize(rows, metrics):
    """Per scenario: run count, successes, and stats of each metric over the
    successful runs (failed runs only count against the success rate)."""
    out = {}
    for scenario in dict.fromkeys(r['scenario'] for r in rows):
        runs = [r for r in rows if r['scenario'] == scenario]
        ok = [r for r in runs if r['result'] == 'complete']
        out[scenario] = {'runs': len(runs), 'ok': len(ok),
                         'stats': {m: _stats([r.get(m) for r in ok]) for m in metrics}}
    return out


def box_distance(px, py, box):
    cx, cy, hx, hy = box
    return math.hypot(max(abs(px - cx) - hx, 0.0), max(abs(py - cy) - hy, 0.0))


def map_quality(occupied, boxes=None, tol=0.15, step=0.1, limit=7.85):
    """How well a built map matches the known layout.

    occupied: [(x, y)] centres of occupied map cells, in the site frame.
    precision: fraction of occupied cells within `tol` of a known box (the
      rest are phantom walls or a misaligned map).
    recall: fraction of box outline points (every `step` m, inside the
      site) with an occupied cell within `tol` - faces the LiDAR never saw,
      and objects lower than the scan plane, count as missed.
    per_box: recall for each box name."""
    named = OBSTACLES if boxes is None else boxes
    near = [any(box_distance(x, y, b) <= tol for b in named.values()) for x, y in occupied]
    precision = sum(near) / len(near) if near else None
    grid = {}
    for x, y in occupied:
        grid.setdefault((round(x / tol), round(y / tol)), []).append((x, y))

    def hit(px, py):
        gx, gy = round(px / tol), round(py / tol)
        return any(math.hypot(px - x, py - y) <= tol
                   for i in (-1, 0, 1) for j in (-1, 0, 1) for x, y in grid.get((gx + i, gy + j), ()))
    per_box, total, found = {}, 0, 0
    for name, (cx, cy, hx, hy) in named.items():
        pts = []
        nx, ny = max(int(2 * hx / step), 1), max(int(2 * hy / step), 1)
        for k in range(nx + 1):
            x = cx - hx + 2 * hx * k / nx
            pts += [(x, cy - hy), (x, cy + hy)]
        for k in range(1, ny):
            y = cy - hy + 2 * hy * k / ny
            pts += [(cx - hx, y), (cx + hx, y)]
        pts = [(x, y) for x, y in pts if abs(x) <= limit and abs(y) <= limit]
        if not pts:
            continue
        h = sum(hit(x, y) for x, y in pts)
        per_box[name] = h / len(pts)
        total += len(pts)
        found += h
    return {'precision': precision, 'recall': found / total if total else None, 'per_box': per_box,
            'occupied_cells': len(occupied)}
