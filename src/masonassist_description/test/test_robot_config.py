import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from robot_config import DEFAULTS, load_robot_config  # noqa: E402


def test_shipped_robot_yaml_is_the_simulated_robot():
    assert load_robot_config() == DEFAULTS


def test_partial_file_overrides_only_what_it_sets(tmp_path):
    f = tmp_path / 'robot.yaml'
    f.write_text('body:\n  half_width_m: 0.5\nsafety:\n  max_linear: 0.4\n')
    cfg = load_robot_config(str(f))
    assert cfg['body']['half_width_m'] == 0.5 and cfg['safety']['max_linear'] == 0.4
    assert cfg['body']['front_m'] == DEFAULTS['body']['front_m']
    assert cfg['lidar'] == DEFAULTS['lidar']


def test_environment_variable_selects_the_file(tmp_path, monkeypatch):
    f = tmp_path / 'other.yaml'
    f.write_text('lidar:\n  x_offset_m: 0.12\n')
    monkeypatch.setenv('MASONASSIST_ROBOT_CONFIG', str(f))
    assert load_robot_config()['lidar']['x_offset_m'] == 0.12


def test_modules_take_their_constants_from_the_config():
    import live_map
    import safety_supervisor_core as sup
    cfg = load_robot_config()
    assert sup.BODY_HALF_WIDTH_M == cfg['body']['half_width_m']
    assert sup.MAX_LINEAR == cfg['safety']['max_linear']
    assert list(map(list, sup.SELF_BANDS_DEG)) == cfg['lidar']['self_occlusion_bands_deg']
    assert live_map.LIDAR_X_OFFSET == sup.LIDAR_X_OFFSET == cfg['lidar']['x_offset_m']
