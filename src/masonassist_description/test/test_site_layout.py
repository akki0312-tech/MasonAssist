import math
import os
import sys
import xml.etree.ElementTree as ET

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


def test_world_parses_and_has_the_site_facilities():
    names = {m.get('name') for m in ET.parse(WORLD).getroot().iter('model')}
    for n in ('site_office', 'concrete_mixer', 'formwork_stack', 'generator', 'skip_bin'):
        assert n in names and n in OBSTACLES
