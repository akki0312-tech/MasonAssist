#!/usr/bin/env python3
"""The site the robot works on (ROS-free): obstacles, stations, missions.

Everything that needs to know where the site's fixed obstacles are - the
route planner, the "unmapped return = possible person" check, the live
map's known-layout mask, goal and station validation, the dashboard - asks
SITE.clearance(x, y): distance to the nearest known obstacle. Two sources:

  LayoutSite  the simulation's authored obstacle boxes (worker_mover.OBSTACLES)
  MapSite     a SLAM map (occupied and never-seen cells count as obstacles)
              plus keep-out boxes for what a 2D LiDAR cannot map

config/site.yaml chooses the source and holds the keep-outs, stations and
missions. Overrides: MASONASSIST_SITE_CONFIG (file), MASONASSIST_SITE_SOURCE
('layout' | 'map').
"""

import os

import numpy as np
import yaml

from site_config import load_no_go_zones, site_config_path, zones_blocking  # noqa: F401

PGM_FREE = 254                  # map_saver trinary values: 0 occupied, 205 unknown


def _box_clearance(xs, ys, boxes):
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    b = np.asarray(list(boxes), dtype=float).reshape(-1, 4)
    if not len(b):
        return np.full(xs.shape, np.inf)
    dx = np.maximum(np.abs(xs[..., None] - b[:, 0]) - b[:, 2], 0.0)
    dy = np.maximum(np.abs(ys[..., None] - b[:, 1]) - b[:, 3], 0.0)
    return np.hypot(dx, dy).min(axis=-1)


class LayoutSite:
    """Obstacles = axis-aligned boxes (cx, cy, hx, hy)."""

    source = 'layout'

    def __init__(self, boxes):
        self.boxes = dict(boxes)

    def clearance(self, xs, ys):
        return _box_clearance(xs, ys, self.boxes.values())

    def draw_items(self):
        return {'boxes': [{'name': n, 'cx': cx, 'cy': cy, 'hx': hx, 'hy': hy}
                          for n, (cx, cy, hx, hy) in self.boxes.items()], 'cells': [], 'cell_size': 0}


def read_pgm_map(yaml_path):
    """(image as uint8 rows top-down, resolution, origin x, origin y)."""
    meta = yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5', 'expected a binary PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return img, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


class MapSite:
    """Obstacles = SLAM map cells that are occupied or were never seen, plus
    keep-out boxes. clearance() looks up a precomputed distance field."""

    source = 'map'

    def __init__(self, img, res, ox, oy, keepouts=None, erase=()):
        from scipy.ndimage import distance_transform_edt
        self.res, self.ox, self.oy = res, ox, oy
        self.h, self.w = img.shape
        self.keepouts = dict(keepouts or {})
        obstacle = img != PGM_FREE
        self.occupied = img == 0
        for x, y in erase:                                   # map cleanup (config/site.yaml)
            row, col = self._cell(x, y)
            if 0 <= row < self.h and 0 <= col < self.w:
                obstacle[row, col] = self.occupied[row, col] = False
        if self.keepouts:
            ys = oy + (self.h - 1 - np.arange(self.h) + 0.5) * res
            xs = ox + (np.arange(self.w) + 0.5) * res
            gx, gy = np.meshgrid(xs, ys)
            obstacle |= _box_clearance(gx, gy, self.keepouts.values()) == 0.0
        self.obstacle = obstacle
        self.field = distance_transform_edt(~obstacle) * res

    @classmethod
    def from_yaml(cls, yaml_path, keepouts=None, erase=()):
        return cls(*read_pgm_map(yaml_path), keepouts=keepouts, erase=erase)

    def _cell(self, xs, ys):
        col = np.floor((np.asarray(xs, dtype=float) - self.ox) / self.res).astype(int)
        row = self.h - 1 - np.floor((np.asarray(ys, dtype=float) - self.oy) / self.res).astype(int)
        return row, col

    def clearance(self, xs, ys):
        xs = np.asarray(xs, dtype=float)
        row, col = self._cell(xs, ys)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.zeros(xs.shape)                          # off the map = unknown = obstacle
        out[inside] = self.field[row[inside], col[inside]]
        return out

    def draw_items(self, cell=0.1):
        """Occupied map cells downsampled to `cell` m, plus the keep-outs."""
        k = max(int(round(cell / self.res)), 1)
        occ = (self.obstacle[:self.h // k * k, :self.w // k * k]
               .reshape(self.h // k, k, self.w // k, k).any(axis=(1, 3)))
        rows, cols = np.nonzero(occ)
        cells = [[round(self.ox + (c + 0.5) * k * self.res, 2), round(self.oy + (self.h - (r + 0.5) * k) * self.res, 2)]
                 for r, c in zip(rows.tolist(), cols.tolist())]
        return {'boxes': [{'name': n, 'cx': cx, 'cy': cy, 'hx': hx, 'hy': hy, 'keepout': True}
                          for n, (cx, cy, hx, hy) in self.keepouts.items()],
                'cells': cells, 'cell_size': round(k * self.res, 3)}


def load_site(config_path=None, source=None):
    """(site model, config dict) from config/site.yaml (or the overrides)."""
    config_path = site_config_path(config_path)
    cfg = yaml.safe_load(open(config_path))
    source = source or os.environ.get('MASONASSIST_SITE_SOURCE') or cfg.get('source', 'layout')
    keepouts = {n: tuple(b) for n, b in (cfg.get('keepouts') or {}).items()}
    if source == 'map':
        map_path = cfg['map']
        if not os.path.isabs(map_path):
            map_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(config_path))), map_path)
        site = MapSite.from_yaml(map_path, keepouts, [tuple(p) for p in cfg.get('erase') or ()])
    elif source == 'layout':
        from worker_mover import OBSTACLES
        site = LayoutSite(OBSTACLES)
    else:
        raise ValueError(f"site source must be 'map' or 'layout', not {source!r}")
    return site, cfg


SITE, SITE_CONFIG = load_site()
BOUNDS = tuple(SITE_CONFIG.get('bounds', (-8.0, 8.0)))
STATIONS = {n: (float(v[0]), float(v[1]), str(v[2])) for n, v in SITE_CONFIG['stations'].items()}
MISSIONS = {n: {'material': m['material'], 'legs': [tuple(leg) for leg in m['legs']]}
            for n, m in SITE_CONFIG['missions'].items()}
