#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('ydlidar_params_file')
    params_declare = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS 2 parameter file to use.',
    )
    publish_tf_declare = DeclareLaunchArgument(
        'publish_static_tf', default_value='false',
        description='Publish a sensor transform for standalone use')
    tf_parent_frame_declare = DeclareLaunchArgument(
        'static_tf_parent_frame', default_value='base_link',
        description='Parent frame for the optional standalone transform')
    tf_child_frame_declare = DeclareLaunchArgument(
        'static_tf_child_frame', default_value='laser_frame',
        description='Child frame; keep this equal to the driver frame_id')
    respawn_declare = DeclareLaunchArgument(
        'respawn', default_value='false',
        description='Restart the driver up to three times after an unexpected exit')

    driver_node = Node(package='ydlidar_ros2_driver',
                       executable='ydlidar_ros2_driver_node',
                       name='ydlidar_ros2_driver_node',
                       output='screen',
                       emulate_tty=True,
                       parameters=[parameter_file],
                       namespace='/',
                       respawn=LaunchConfiguration('respawn'),
                       respawn_delay=2.0,
                       respawn_max_retries=3,
                       )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=[
                        '--x', '0', '--y', '0', '--z', '0.02',
                        '--yaw', '3.14159265',
                        '--frame-id', LaunchConfiguration('static_tf_parent_frame'),
                        '--child-frame-id', LaunchConfiguration('static_tf_child_frame'),
                    ],
                    condition=IfCondition(LaunchConfiguration('publish_static_tf')),
                    )

    return LaunchDescription([
        params_declare,
        publish_tf_declare,
        tf_parent_frame_declare,
        tf_child_frame_declare,
        respawn_declare,
        driver_node,
        tf2_node,
    ])
