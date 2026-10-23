#!/usr/bin/env python3
"""Payload for the MasonAssist model (ROS-free).

payload_link_sdf(mass) returns the SDF for a loaded pallet on the robot's
deck - a 'payload' link with the given mass and solid-box inertia, fixed to
base_link - for insertion into models/masonassist/model.sdf at spawn time.
The load is a block of stacked bricks/blocks whose height follows from its
mass, so a heavier load also raises the centre of gravity.
"""

BASE_LINK_Z = 0.07683          # base_link pose in the model frame (model.sdf)
DECK_TOP_Z = 0.225             # top of the payload deck, base_link frame (deck visual 0.195 +- 0.03)
LOAD_X, LOAD_Y = 0.50, 0.36    # load footprint, inside the 0.55 x 0.40 m deck
PALLET_H = 0.04
BULK_DENSITY = 1600.0          # kg/m^3, stacked bricks / concrete blocks with gaps
MIN_LOAD_H = 0.03


def load_height(mass):
    return max(mass / (LOAD_X * LOAD_Y * BULK_DENSITY), MIN_LOAD_H)


def box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y))


def payload_cog(mass, cog_height=None):
    """(height of the load's centre above the deck top, load height)."""
    h = load_height(mass)
    return (PALLET_H + h / 2.0 if cog_height is None else cog_height), h


def payload_link_sdf(mass, cog_height=None):
    """SDF <link>+<joint> for `mass` kg on the deck, or '' for no payload.
    cog_height: centre of the load above the deck top (default: resting on
    the pallet)."""
    if mass <= 0:
        return ''
    above, h = payload_cog(mass, cog_height)
    z = BASE_LINK_Z + DECK_TOP_Z + above          # load centre, model frame
    pallet_z = BASE_LINK_Z + DECK_TOP_Z + PALLET_H / 2.0
    ixx, iyy, izz = box_inertia(mass, LOAD_X, LOAD_Y, h)
    load = f'<box><size>{LOAD_X} {LOAD_Y} {h:.4f}</size></box>'
    return f"""
    <link name="payload">
      <pose>0 0 {z:.4f} 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.5f}</iyy><iyz>0</iyz><izz>{izz:.5f}</izz></inertia>
      </inertial>
      <collision name="load_collision"><geometry>{load}</geometry></collision>
      <visual name="load_visual"><geometry>{load}</geometry>
        <material><ambient>0.65 0.25 0.15 1</ambient><diffuse>0.65 0.25 0.15 1</diffuse></material></visual>
      <visual name="pallet_visual"><pose>0 0 {pallet_z - z:.4f} 0 0 0</pose>
        <geometry><box><size>0.54 0.39 {PALLET_H}</size></box></geometry>
        <material><ambient>0.45 0.32 0.18 1</ambient><diffuse>0.45 0.32 0.18 1</diffuse></material></visual>
    </link>
    <joint name="payload_joint" type="fixed">
      <parent>base_link</parent>
      <child>payload</child>
    </joint>
"""


def model_with_payload(model_sdf_text, mass, cog_height=None):
    """The robot model SDF with the payload inserted before </model>."""
    extra = payload_link_sdf(mass, cog_height)
    if not extra:
        return model_sdf_text
    head, sep, tail = model_sdf_text.rpartition('</model>')
    assert sep, 'no </model> in the robot model'
    return head + extra + sep + tail
