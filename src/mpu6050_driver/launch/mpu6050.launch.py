import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpu6050_driver')
    params_file = os.path.join(pkg_dir, 'config', 'mpu6050_params.yaml')

    mpu6050_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    # Static TF: base_link → imu_link
    # NOTE: When using URDF (robot_state_publisher), this TF is already
    # defined in the URDF. Only launch this when robot_state_publisher
    # is NOT running (e.g., standalone IMU testing).
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.03',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    return LaunchDescription([
        mpu6050_node,
        # imu_tf is commented out because URDF handles this TF
        # Uncomment imu_tf below for standalone testing without URDF:
        # imu_tf,
    ])
