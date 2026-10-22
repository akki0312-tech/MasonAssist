import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from worker_mover import OBSTACLES  # noqa: E402

WORLD = os.path.join(os.path.dirname(__file__), "..", "worlds", "construction_site.world")
# collision models that are not obstacles: the floor, the drivable slab, the
# moving workers (handled live by the safety zones)
NOT_OBSTACLES = {'ground_plane', 'concrete_slab', 'worker_1', 'worker_2'}


def _pose(el):
    p = el.find('pose')
    return [float(v) for v in p.text.split()] if p is not None else [0.0] * 6


def _collision_points():
    """(model, x, y) world position of every static collision geometry."""
    for model in ET.parse(WORLD).getroot().iter('model'):
        name = model.get('name')
        if name in NOT_OBSTACLES:
            continue
        mx, my, _, _, _, myaw = _pose(model)
        for col in model.iter('collision'):
            cx, cy = _pose(col)[:2]
            yield (name,
                   mx + cx * math.cos(myaw) - cy * math.sin(myaw),
                   my + cx * math.sin(myaw) + cy * math.cos(myaw))


def test_every_collision_object_in_the_world_is_in_the_obstacle_registry():
    # The planner, safety zones and live map only know OBSTACLES: an object
    # missing from it would be planned through and treated as unknown debris.
    missing = [(n, round(x, 2), round(y, 2)) for n, x, y in _collision_points()
               if not any(abs(x - cx) <= hx and abs(y - cy) <= hy
                          for cx, cy, hx, hy in OBSTACLES.values())]
    assert not missing


# Height band the robot can hit or its LiDAR (~0.27 m) can see; ground decals
# below it and overhead parts (scaffold platforms, signs) above it are ignored.
BAND_Z = (0.02, 0.5)
MATCH_M = 0.1


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _half_extents(geom):
    box = geom.find('box/size')
    if box is not None:
        return np.array([float(v) for v in box.text.split()]) / 2
    for kind in ('cylinder', 'cone'):
        g = geom.find(kind)
        if g is not None:
            r = float(g.find('radius').text)
            return np.array([r, r, float(g.find('length').text) / 2])
    sph = geom.find('sphere/radius')
    if sph is not None:
        return np.full(3, float(sph.text))
    return None   # plane / mesh: not used by the obstacles


def _shape_corners():
    """(model, kind/name, 8x3 world corners) of every visual and collision
    shape of every obstacle model, through model, link and shape poses."""
    for model in ET.parse(WORLD).getroot().iter('model'):
        name = model.get('name')
        if name in NOT_OBSTACLES or model.find('.//collision') is None:
            continue
        mp = _pose(model)
        Rm, tm = _rot(*mp[3:]), np.array(mp[:3])
        for link in model.iter('link'):
            lp = _pose(link)
            Rl, tl = Rm @ _rot(*lp[3:]), tm + Rm @ np.array(lp[:3])
            for shape in list(link.iter('visual')) + list(link.iter('collision')):
                h = _half_extents(shape.find('geometry'))
                if h is None:
                    continue
                sp = _pose(shape)
                Rs, ts = Rl @ _rot(*sp[3:]), tl + Rl @ np.array(sp[:3])
                signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
                yield name, f'{shape.tag}/{shape.get("name")}', ts + (signs * h) @ Rs.T


def test_registry_covers_every_shape_the_robot_can_hit_or_see():
    # Catches boxes that are too small for the model, e.g. a model rolled
    # 90 degrees whose pipes lie along a different axis than its box.
    boxes = np.array(list(OBSTACLES.values()))
    uncovered = []
    for model, shape, c in _shape_corners():
        if c[:, 2].max() < BAND_Z[0] or c[:, 2].min() > BAND_Z[1]:
            continue
        lo, hi = c[:, :2].min(axis=0), c[:, :2].max(axis=0)
        xs = np.linspace(lo[0], hi[0], max(int((hi[0] - lo[0]) / 0.05), 1) + 1)
        ys = np.linspace(lo[1], hi[1], max(int((hi[1] - lo[1]) / 0.05), 1) + 1)
        pts = np.array([(x, y) for x in xs for y in ys])
        dx = np.maximum(np.abs(pts[:, None, 0] - boxes[:, 0]) - boxes[:, 2], 0)
        dy = np.maximum(np.abs(pts[:, None, 1] - boxes[:, 1]) - boxes[:, 3], 0)
        worst = np.hypot(dx, dy).min(axis=1).max()
        if worst > MATCH_M:
            uncovered.append((model, shape, round(float(worst), 2)))
    assert not uncovered


def test_world_parses_and_has_the_site_facilities():
    names = {m.get('name') for m in ET.parse(WORLD).getroot().iter('model')}
    for n in ('site_office', 'concrete_mixer', 'formwork_stack', 'generator', 'skip_bin'):
        assert n in names and n in OBSTACLES
