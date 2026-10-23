import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from site_model import LayoutSite, MapSite, MISSIONS, STATIONS, load_site  # noqa: E402
from worker_mover import OBSTACLES  # noqa: E402

PKG = os.path.join(os.path.dirname(__file__), "..")


@pytest.fixture(scope='module')
def map_site():
    return load_site(source='map')[0]


def _old_box_clearance(x, y):
    best = math.inf
    for cx, cy, hx, hy in OBSTACLES.values():
        best = min(best, math.hypot(max(abs(x - cx) - hx, 0.0), max(abs(y - cy) - hy, 0.0)))
    return best


def test_layout_site_matches_the_old_box_maths():
    site = LayoutSite(OBSTACLES)
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(-8, 8, size=(300, 2)):
        assert site.clearance(x, y) == pytest.approx(_old_box_clearance(x, y))


def test_map_site_on_a_synthetic_map():
    img = np.full((40, 40), 254, dtype=np.uint8)        # 2 x 2 m of free floor at 0.05 m
    img[:, 20] = 0                                       # a wall at x = 1.0..1.05
    img[0:5, 0:5] = 205                                  # an unseen corner (top-left)
    site = MapSite(img, 0.05, 0.0, 0.0, keepouts={'k': (0.25, 0.25, 0.1, 0.1)})
    assert site.clearance(0.5, 1.0) == pytest.approx(0.5, abs=0.05)   # half a metre from the wall
    assert site.clearance(1.02, 1.0) == 0.0                            # on the wall
    assert site.clearance(0.1, 1.9) == 0.0                             # unseen counts as obstacle
    assert site.clearance(0.25, 0.25) == 0.0                           # keep-out
    assert site.clearance(5.0, 5.0) == 0.0                             # off the map
    assert site.draw_items()['cells']


def test_no_known_object_is_passable_on_the_map(map_site):
    # the largest map clearance inside every known object stays below 0.5 m,
    # so the planner (which needs 0.7 m) can never route through one -
    # the keep-outs in config/site.yaml cover the ones the LiDAR can't map
    for name, (cx, cy, hx, hy) in OBSTACLES.items():
        xs, ys = np.meshgrid(np.arange(cx - hx, cx + hx + 1e-9, 0.05), np.arange(cy - hy, cy + hy + 1e-9, 0.05))
        assert map_site.clearance(xs, ys).max() < 0.5, name


@pytest.mark.parametrize('source', ['layout', 'map'])
def test_every_station_and_mission_leg_works_in_both_modes(source):
    from site_planner import SitePlanner
    from transport_mission import MIN_STATION_CLEARANCE_M
    site = load_site(source=source)[0]
    for name, (x, y, _) in STATIONS.items():
        assert site.clearance(x, y) >= MIN_STATION_CLEARANCE_M, name
    planner = SitePlanner(site=site)
    for spec in MISSIONS.values():
        pos = STATIONS['dock'][:2]
        for station, _ in spec['legs']:
            goal = STATIONS[station][:2]
            assert planner.plan(pos, goal) is not None, (source, station)
            pos = goal


def test_site_config_overrides(tmp_path, monkeypatch):
    monkeypatch.setenv('MASONASSIST_SITE_SOURCE', 'layout')
    site, cfg = load_site()
    assert site.source == 'layout' and 'dock' in cfg['stations']
    with pytest.raises(ValueError):
        load_site(source='nonsense')


def test_map_has_no_phantom_obstacles_left(map_site):
    # every remaining occupied cell is within 0.3 m of a real object (the
    # erase list removed the SLAM speckle); unknown cells are obstacles by design
    from site_model import _box_clearance
    rows, cols = np.nonzero(map_site.occupied)
    xs = map_site.ox + (cols + 0.5) * map_site.res
    ys = map_site.oy + (map_site.h - 1 - rows + 0.5) * map_site.res
    inside = (np.abs(xs) < 7.6) & (np.abs(ys) < 7.6)
    far = _box_clearance(xs[inside], ys[inside], list(OBSTACLES.values()) + list(map_site.keepouts.values())) > 0.3
    assert not far.any(), list(zip(xs[inside][far].round(2), ys[inside][far].round(2)))[:10]


def test_every_low_object_is_a_keep_out(map_site):
    # a 2D LiDAR at 0.27 m sees at most the top of anything lower than 0.5 m,
    # so the map alone under-represents its footprint (measured: the robot
    # passed 2 cm from rubble in map mode before rubble became a keep-out)
    sys.path.insert(0, os.path.dirname(__file__))
    import test_site_layout as layout
    tops = {}
    for _model, shape, c in layout._shape_corners():
        if not shape.startswith('collision'):
            continue
        cx, cy = c[:, 0].mean(), c[:, 1].mean()
        for name, (bx, by, hx, hy) in OBSTACLES.items():
            if abs(cx - bx) <= hx + 0.05 and abs(cy - by) <= hy + 0.05:
                tops[name] = max(tops.get(name, 0.0), c[:, 2].max())
    low = [n for n, top in tops.items() if top < 0.5]
    assert low and all(n in map_site.keepouts for n in low), low
