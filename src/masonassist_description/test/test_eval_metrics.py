import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from eval_metrics import (  # noqa: E402
    arrival_errors, body_clearance, footprint, path_length, planned_length,
    polygon_distance, route_length, summarize,
)
from safety_supervisor_core import BODY_FRONT_M, BODY_HALF_WIDTH_M  # noqa: E402


def test_path_length_ignores_jitter_but_counts_travel():
    still = [(0.0, 0.0), (0.004, 0.0), (0.0, 0.003), (0.002, 0.001)] * 50
    assert path_length(still) == 0.0
    line = [(i * 0.1, 0.0) for i in range(11)]
    assert path_length(line) == pytest.approx(1.0)
    # an L-shaped path with jitter on top
    l_path = [(0, 0), (1, 0), (1.003, 0.002), (1, 1)]
    assert path_length(l_path) == pytest.approx(2.0, abs=0.01)


def test_route_and_planned_length():
    assert route_length((0, 0), [(3, 0), (3, 4)]) == pytest.approx(7.0)

    class Straight:
        def plan(self, start, goal):
            return [goal]
    assert planned_length((0, 0), [(3, 0), (3, 4), (0, 0)], Straight()) == pytest.approx(12.0)

    class Nowhere:
        def plan(self, start, goal):
            return None
    assert planned_length((0, 0), [(1, 0)], Nowhere()) is None


def test_footprint_corners_rotate_with_heading():
    c = footprint(0.0, 0.0, 0.0)
    assert max(p[0] for p in c) == pytest.approx(BODY_FRONT_M)
    c = footprint(1.0, 2.0, math.pi / 2)        # facing +y: front edge at y = 2 + front
    assert max(p[1] for p in c) == pytest.approx(2.0 + BODY_FRONT_M)
    assert max(p[0] for p in c) == pytest.approx(1.0 + BODY_HALF_WIDTH_M)


def test_body_clearance_known_answers():
    box_ahead = [(2.0, 0.0, 0.5, 0.5)]          # near face at x = 1.5
    assert body_clearance(0, 0, 0, box_ahead) == pytest.approx(1.5 - BODY_FRONT_M)
    box_left = [(0.0, 2.0, 0.5, 0.5)]           # near face at y = 1.5
    assert body_clearance(0, 0, 0, box_left) == pytest.approx(1.5 - BODY_HALF_WIDTH_M)
    # turned 90 deg, the front now faces the box on the left
    assert body_clearance(0, 0, math.pi / 2, box_left) == pytest.approx(1.5 - BODY_FRONT_M)
    assert body_clearance(1.6, 0, 0, box_ahead) == 0.0     # overlapping


def test_body_clearance_diagonal_corner():
    # box corner at (1, 1); robot corner at (front, half width)
    d = body_clearance(0, 0, 0, [(1.5, 1.5, 0.5, 0.5)])
    assert d == pytest.approx(math.hypot(1 - BODY_FRONT_M, 1 - BODY_HALF_WIDTH_M))


def test_polygon_distance_symmetric():
    a = [(0, 0), (1, 0), (1, 1), (0, 1)]
    b = [(3, 0), (4, 0), (4, 1), (3, 1)]
    assert polygon_distance(a, b) == pytest.approx(2.0) == polygon_distance(b, a)


def test_arrival_errors_on_station_phases():
    targets = [(0.0, 0.0), (5.0, 0.0), (0.0, 0.0)]
    trace = [
        {'x': 2.0, 'y': 0, 'phase': 'traveling', 'leg_index': 0},
        {'x': 0.2, 'y': 0, 'phase': 'loading', 'leg_index': 0},
        {'x': 0.2, 'y': 0, 'phase': 'loading', 'leg_index': 0},       # same phase: not counted again
        {'x': 3.0, 'y': 0, 'phase': 'traveling', 'leg_index': 1},
        {'x': 4.7, 'y': 0, 'phase': 'unloading', 'leg_index': 1},
        {'x': 2.0, 'y': 0, 'phase': 'traveling', 'leg_index': 2},
        {'x': 0.0, 'y': 0.25, 'phase': 'complete', 'leg_index': 3},   # index past the end -> last leg
    ]
    assert arrival_errors(trace, targets) == pytest.approx([0.2, 0.3, 0.25])


def test_summarize_counts_failures_against_success_only():
    rows = [
        {'scenario': 'bricks', 'result': 'complete', 'wall_s': 150.0},
        {'scenario': 'bricks', 'result': 'complete', 'wall_s': 160.0},
        {'scenario': 'bricks', 'result': 'failed', 'wall_s': 900.0},
        {'scenario': 'goals', 'result': 'complete', 'wall_s': None},
    ]
    s = summarize(rows, ['wall_s'])
    assert s['bricks']['runs'] == 3 and s['bricks']['ok'] == 2
    st = s['bricks']['stats']['wall_s']
    assert st['mean'] == pytest.approx(155.0) and st['max'] == 160.0
    assert st['sd'] == pytest.approx(math.sqrt(50))
    assert s['goals']['stats']['wall_s'] is None


def test_map_quality_perfect_partial_and_phantom():
    from eval_metrics import map_quality
    boxes = {'a': (0.0, 0.0, 1.0, 0.5)}
    outline = [(x / 10, y) for x in range(-10, 11) for y in (-0.5, 0.5)] + \
              [(x, y / 10) for x in (-1.0, 1.0) for y in range(-5, 6)]
    q = map_quality(outline, boxes)
    assert q['precision'] == 1.0 and q['recall'] == pytest.approx(1.0)
    # only the south face seen, plus two phantom cells far away
    south = [(x / 10, -0.5) for x in range(-10, 11)]
    q = map_quality(south + [(5.0, 5.0), (-5.0, 3.0)], boxes)
    assert q['precision'] == pytest.approx(21 / 23)
    assert 0.3 < q['recall'] < 0.5
    assert map_quality([], boxes)['precision'] is None
