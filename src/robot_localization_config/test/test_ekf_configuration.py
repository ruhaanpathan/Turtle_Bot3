from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_wheel_yaw_is_the_only_configured_yaw_source():
    with (PACKAGE_ROOT / 'config' / 'ekf.yaml').open() as stream:
        parameters = yaml.safe_load(stream)['ekf_filter_node']['ros__parameters']

    enabled_odom_states = {
        index for index, enabled in enumerate(parameters['odom0_config'])
        if enabled
    }
    assert enabled_odom_states == {6, 11}  # forward velocity and yaw rate
    assert 'imu0' not in parameters
    assert parameters['publish_tf'] is True
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'
    assert parameters['world_frame'] == 'odom'
