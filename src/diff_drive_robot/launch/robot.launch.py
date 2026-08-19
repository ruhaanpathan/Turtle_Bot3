import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_pkg_dir = get_package_share_directory('diff_drive_robot')
    robot_params = os.path.join(robot_pkg_dir, 'config', 'params.yaml')
    xacro_file = os.path.join(robot_pkg_dir, 'urdf', 'robot.urdf.xacro')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': use_sim_time,
        }],
    )

    mpu_pkg_dir = get_package_share_directory('mpu6050_driver')
    mpu_params = os.path.join(mpu_pkg_dir, 'config', 'mpu6050_params.yaml')

    robot_node = Node(
        package='diff_drive_robot',
        executable='diff_drive_robot',
        output='screen',
        parameters=[robot_params, {'use_sim_time': use_sim_time}],
    )

    mpu_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        emulate_tty=True,
        parameters=[mpu_params, {'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        robot_node,
        mpu_node,
    ])
