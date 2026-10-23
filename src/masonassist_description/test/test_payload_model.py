import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from payload_model import (  # noqa: E402
    BASE_LINK_Z, DECK_TOP_Z, PALLET_H, box_inertia, load_height, model_with_payload,
    payload_link_sdf,
)

MODEL = os.path.join(os.path.dirname(__file__), "..", "models", "masonassist", "model.sdf")


def test_no_payload_no_link():
    assert payload_link_sdf(0) == ''
    text = open(MODEL).read()
    assert model_with_payload(text, 0) == text


def test_load_height_follows_mass():
    assert load_height(50) == pytest.approx(50 / (0.5 * 0.36 * 1600))
    assert load_height(50) > load_height(25) > load_height(15)
    assert load_height(1) == 0.03                     # minimum visible height


def test_box_inertia_known_values():
    ixx, iyy, izz = box_inertia(12.0, 1.0, 2.0, 3.0)
    assert (ixx, iyy, izz) == pytest.approx((13.0, 10.0, 5.0))


def test_payload_link_mass_pose_and_joint():
    root = ET.fromstring('<model>' + payload_link_sdf(25) + '</model>')
    link = root.find('link')
    assert link.get('name') == 'payload'
    assert float(link.find('inertial/mass').text) == 25
    z = float(link.find('pose').text.split()[2])
    assert z == pytest.approx(BASE_LINK_Z + DECK_TOP_Z + PALLET_H + load_height(25) / 2, abs=1e-4)
    joint = root.find('joint')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').text == 'base_link' and joint.find('child').text == 'payload'


def test_raised_centre_of_gravity():
    root = ET.fromstring('<model>' + payload_link_sdf(25, cog_height=0.4) + '</model>')
    z = float(root.find('link/pose').text.split()[2])
    assert z == pytest.approx(BASE_LINK_Z + DECK_TOP_Z + 0.4, abs=1e-4)


def test_model_with_payload_is_valid_and_adds_mass():
    root = ET.fromstring(model_with_payload(open(MODEL).read(), 50))
    masses = {l.get('name'): float(l.find('inertial/mass').text) for l in root.iter('link')
              if l.find('inertial/mass') is not None}
    assert masses['payload'] == 50
    assert sum(masses.values()) == pytest.approx(150.0)   # 80 + 2 x 10 + 50
