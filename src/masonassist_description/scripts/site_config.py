#!/usr/bin/env python3
"""Where config/site.yaml is, and its no-go zones (ROS-free, light: no map).

No-go zones are areas a robot loaded above a zone's max_payload_kg must not
enter - e.g. the top of a steep descent, where the terrain tests showed a
50 kg load tipping the robot over the crest before any tilt sensor could
react. The route planner avoids them and the safety supervisor refuses to
drive into them.
"""

import os

import yaml


def _package_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..'),                                    # source tree
                      os.path.join(here, '..', '..', 'share', 'masonassist_description')):
        if os.path.exists(os.path.join(candidate, 'config', 'site.yaml')):
            return os.path.normpath(candidate)
    raise FileNotFoundError('config/site.yaml not found next to the scripts or in the install share')


def site_config_path(path=None):
    return path or os.environ.get('MASONASSIST_SITE_CONFIG') or \
        os.path.join(_package_dir(), 'config', 'site.yaml')


def load_no_go_zones(path=None):
    """{name: {'box': (cx, cy, hx, hy), 'max_payload_kg': float, 'reason': str}}"""
    cfg = yaml.safe_load(open(site_config_path(path))) or {}
    return {name: {'box': tuple(float(v) for v in z['box']),
                   'max_payload_kg': float(z.get('max_payload_kg', 0.0)),
                   'reason': z.get('reason', '')}
            for name, z in (cfg.get('no_go_zones') or {}).items()}


def zones_blocking(zones, payload_kg):
    """The zones that apply to a robot carrying payload_kg: {name: box}."""
    return {name: z['box'] for name, z in zones.items() if payload_kg > z['max_payload_kg']}
