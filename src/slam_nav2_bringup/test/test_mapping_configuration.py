from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_parameters(filename, node_name):
    with (PACKAGE_ROOT / 'config' / filename).open() as stream:
        return yaml.safe_load(stream)[node_name]['ros__parameters']


def test_mapping_turns_are_limited_for_x2_scan_rate():
    navigation = load_parameters('nav2_params.yaml', 'velocity_smoother')
    behavior = load_parameters('nav2_params.yaml', 'behavior_server')
    controller = load_parameters('nav2_params.yaml', 'controller_server')

    assert navigation['max_velocity'][2] <= 0.4
    assert navigation['min_velocity'][2] >= -0.4
    assert navigation['max_accel'][2] <= 1.0
    assert navigation['max_decel'][2] >= -1.0
    assert behavior['max_rotational_vel'] <= 0.4
    assert behavior['rotational_acc_lim'] <= 1.0
    assert controller['FollowPath']['rotate_to_heading_angular_vel'] <= 0.4


def test_scan_matcher_uses_non_regressed_reference_values():
    slam = load_parameters('slam_params.yaml', 'slam_toolbox')

    assert slam['link_match_minimum_response_fine'] <= 0.1
    assert slam['correlation_search_space_smear_deviation'] >= 0.1
    assert slam['angle_variance_penalty'] >= 1.0
    assert slam['minimum_angle_penalty'] >= 0.9
    assert slam['stack_size_to_use'] >= 40_000_000
