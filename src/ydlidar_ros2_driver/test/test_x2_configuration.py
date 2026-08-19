# Copyright 2026 YDLIDAR ROS 2 driver contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILES = (
    'ydlidar.py',
    'ydlidar_launch.py',
    'ydlidar_launch_view.py',
)


def load_launch_description(filename):
    path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def argument_default(description, name):
    action = next(
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == name
    )
    return perform_substitutions(LaunchContext(), action.default_value)


@pytest.mark.parametrize('filename', ('ydlidar.yaml', 'X2.yaml'))
def test_default_parameters_match_proven_x2_transport_profile(filename):
    with (PACKAGE_ROOT / 'params' / filename).open() as stream:
        parameters = yaml.safe_load(stream)[
            'ydlidar_ros2_driver_node']['ros__parameters']

    expected = {
        'port': '/dev/ydlidar',
        'frame_id': 'laser_frame',
        'baudrate': 115200,
        'lidar_type': 1,
        'device_type': 0,
        'sample_rate': 3,
        'abnormal_check_count': 4,
        'isSingleChannel': True,
        'intensity': False,
        'intensity_bit': 0,
        'auto_intensity': False,
        'auto_reconnect': True,
        'support_motor_dtr': True,
        'frequency': 7.0,
        'fixed_resolution': False,
        'reversion': False,
        # Preserve the handedness of the previously working X2 driver path.
        'inverted': True,
        'angle_min': -180.0,
        'angle_max': 180.0,
        'range_min': 0.12,
        'range_max': 8.0,
        'invalid_range_is_inf': False,
    }
    assert {key: parameters[key] for key in expected} == expected
    assert 0.0 <= parameters['range_min'] < parameters['range_max']
    assert parameters['angle_min'] < parameters['angle_max']


@pytest.mark.parametrize('filename', LAUNCH_FILES)
def test_launch_parses_and_does_not_respawn_by_default(filename):
    description = load_launch_description(filename)
    assert argument_default(description, 'respawn') == 'false'

    driver = next(
        entity for entity in description.entities
        if isinstance(entity, Node) and
        entity.node_executable == 'ydlidar_ros2_driver_node'
    )
    context = LaunchContext()
    context.launch_configurations['respawn'] = 'false'
    assert perform_substitutions(
        context, driver._ExecuteLocal__respawn) == 'false'
    assert driver._ExecuteLocal__respawn_max_retries == 3


@pytest.mark.parametrize('filename', ('ydlidar_launch.py', 'ydlidar_launch_view.py'))
def test_static_transform_is_opt_in(filename):
    description = load_launch_description(filename)
    assert argument_default(description, 'publish_static_tf') == 'false'
    assert argument_default(description, 'static_tf_parent_frame') == 'base_link'
    assert argument_default(description, 'static_tf_child_frame') == 'laser_frame'

    transform = next(
        entity for entity in description.entities
        if isinstance(entity, Node) and
        entity.node_executable == 'static_transform_publisher'
    )
    context = LaunchContext()
    context.launch_configurations['static_tf_parent_frame'] = 'robot_base'
    context.launch_configurations['static_tf_child_frame'] = 'custom_laser'
    arguments = [
        perform_substitutions(
            context, normalize_to_list_of_substitutions(argument))
        for argument in transform._Node__arguments
    ]
    assert arguments[arguments.index('--frame-id') + 1] == 'robot_base'
    assert arguments[arguments.index('--child-frame-id') + 1] == 'custom_laser'
