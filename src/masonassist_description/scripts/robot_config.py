#!/usr/bin/env python3
"""Robot parameters, loaded once from config/robot.yaml (ROS-free).

Override with the environment variable MASONASSIST_ROBOT_CONFIG=/path.yaml.
Keys missing from the file fall back to DEFAULTS (the simulated robot), so
a partial file only needs what differs.
"""

import copy
import os

import yaml

DEFAULTS = {
    'body': {'front_m': 0.3625, 'rear_m': 0.3625, 'half_width_m': 0.43},
    'lidar': {'x_offset_m': 0.30, 'min_range_m': 0.1, 'self_range_m': 0.25,
              'self_occlusion_bands_deg': [[-105.8, -96.8], [96.8, 105.8]]},
    'navigation': {'robot_radius_m': 0.4475, 'safety_margin_m': 0.15,
                   'forward_speed_max': 0.2, 'max_angular_speed': 0.3},
    'safety': {'max_linear': 0.25, 'max_angular': 0.5, 'guard_margin_m': 0.15, 'no_go_lookahead_m': 0.25,
               'tilt_limits': [[25.0, 35.0], [50.0, 22.0]], 'tilt_slow_band_deg': 5.0,
               'tilt_slow_speed': 0.1, 'tilt_release_deg': 2.0},
}


def _default_path():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..', 'config', 'robot.yaml'),            # source tree
                      os.path.join(here, '..', '..', 'share', 'masonassist_description',
                                   'config', 'robot.yaml')):                       # installed
        if os.path.exists(candidate):
            return os.path.normpath(candidate)
    return None


def _merge(base, extra):
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


def load_robot_config(path=None):
    path = path or os.environ.get('MASONASSIST_ROBOT_CONFIG') or _default_path()
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        with open(path) as f:
            _merge(cfg, yaml.safe_load(f))
    return cfg


ROBOT = load_robot_config()
