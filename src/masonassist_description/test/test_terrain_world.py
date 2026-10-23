import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools"))

from make_terrain_world import CROSS_SLOPE_DEG, LANES, RAMP_LEN, SURFACES, lane_y  # noqa: E402

WORLD = os.path.join(os.path.dirname(__file__), "..", "worlds", "terrain_test.world")


def _models():
    out = {}
    for m in ET.parse(WORLD).getroot().iter('model'):
        if m.find('pose') is None:
            continue
        mu = m.find('.//friction/ode/mu')
        out[m.get('name')] = ([float(v) for v in m.find('pose').text.split()],
                              None if mu is None else float(mu.text))
    return out


@pytest.mark.parametrize('i, surface, deg', [(i, s, d) for i, (s, d) in enumerate(LANES)])
def test_ramp_lanes_match_their_spec(i, surface, deg):
    m = _models()
    key = f'{surface}_{deg}'
    (up, mu_up), (down, _), (plateau, _) = m[f'ramp_up_{key}'], m[f'ramp_down_{key}'], m[f'plateau_{key}']
    assert math.degrees(-up[4]) == pytest.approx(deg, abs=1e-3)
    assert math.degrees(down[4]) == pytest.approx(deg, abs=1e-3)
    assert up[1] == down[1] == plateau[1] == lane_y(i)
    assert plateau[2] * 2 == pytest.approx(RAMP_LEN * math.sin(math.radians(deg)), abs=1e-3)
    assert mu_up == SURFACES[surface][0]


def test_cross_slope():
    assert math.degrees(_models()[f'cross_slope_{CROSS_SLOPE_DEG}'][0][3]) == pytest.approx(CROSS_SLOPE_DEG, abs=1e-3)
