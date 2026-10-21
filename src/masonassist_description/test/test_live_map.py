import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from live_map import PERSIST_S, LiveMap  # noqa: E402

N_BEAMS = 360
ANGLE_MIN = -math.pi
INC = 2 * math.pi / N_BEAMS


def _scan(**beams):
    """+inf everywhere except the given {beam_index: range}."""
    r = np.full(N_BEAMS, np.inf)
    for k, v in beams.items():
        r[int(k[1:])] = v
    return r


def _beam(angle):
    return 'b%d' % (int(round((angle - ANGLE_MIN) / INC)) % N_BEAMS)


def _occupied_near(m, x, y, now, radius=0.15):
    return any(math.hypot(cx - x, cy - y) <= radius for cx, cy in m.cells(now))


POSE = (-5.0, -2.0, 0.0)        # open ground, facing +x
TARGET = (-5.0 + 0.30 + 1.5, -2.0)   # 1.5 m beyond the laser (0.30 m ahead of base)


def test_unmapped_obstacle_needs_two_hits_then_persistence():
    m = LiveMap()
    scan = _scan(**{_beam(0.0): 1.5})
    m.update(POSE, scan, ANGLE_MIN, INC, 0.0)
    assert m.logodds.max() < 1.5                         # one hit is not enough
    m.update(POSE, scan, ANGLE_MIN, INC, 0.5)
    assert not _occupied_near(m, *TARGET, now=0.5 + PERSIST_S - 0.1)
    assert _occupied_near(m, *TARGET, now=0.5 + PERSIST_S)


def test_beam_passing_through_clears_the_cell():
    m = LiveMap()
    scan = _scan(**{_beam(0.0): 1.5})
    m.update(POSE, scan, ANGLE_MIN, INC, 0.0)
    m.update(POSE, scan, ANGLE_MIN, INC, 0.5)
    m.update(POSE, _scan(), ANGLE_MIN, INC, 1.0)         # now nothing there: +inf passes through
    assert not _occupied_near(m, *TARGET, now=10.0)


def test_known_wall_is_never_learned():
    m = LiveMap()
    pose = (0.0, 6.0, math.pi / 2)                      # facing wall_north (face at y=7.8)
    scan = _scan(**{_beam(0.0): 7.8 - 6.3})
    for t in range(10):
        m.update(pose, scan, ANGLE_MIN, INC, t * 0.5)
    assert m.cells(100.0) == []


def test_self_returns_and_robot_footprint_ignored():
    m = LiveMap()
    scan = _scan(**{_beam(math.radians(100)): 0.15,     # track housing
                    _beam(math.pi): 0.5})               # lands 0.2 m behind base_link
    for t in range(10):
        m.update(POSE, scan, ANGLE_MIN, INC, t * 0.5)
    assert m.cells(100.0) == []


def test_negative_inf_beams_do_not_clear_space():
    # an obstacle wide enough to cover +-5 beams (a single-beam target is only
    # ~3 cm wide at 1.5 m, so neighbouring beams would legitimately clear it)
    m = LiveMap()
    centre = int(_beam(0.0)[1:])
    wide = np.full(N_BEAMS, np.inf)
    for k in range(centre - 5, centre + 6):
        wide[k] = 1.5 / math.cos((k - centre) * INC)    # flat face 1.5 m ahead of the laser
    m.update(POSE, wide, ANGLE_MIN, INC, 0.0)
    m.update(POSE, wide, ANGLE_MIN, INC, 0.5)
    no_info = wide.copy()
    no_info[centre - 5:centre + 6] = -np.inf             # "too close" readings: no information
    m.update(POSE, no_info, ANGLE_MIN, INC, 1.0)
    assert _occupied_near(m, *TARGET, now=0.5 + PERSIST_S)
