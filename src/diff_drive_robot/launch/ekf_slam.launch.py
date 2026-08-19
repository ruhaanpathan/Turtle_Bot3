from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LiDAR scan topic name'
    )

    landmarks_node = Node(
        package='diff_drive_robot',
        executable='landmarks_node',
        name='landmarks_node',
        output='screen',
        parameters=[{
            'distance_threshold': 0.08,
            'scan_topic': LaunchConfiguration('scan_topic'),
            'body_frame': 'base_link',
            'publish_tf': False,
        }]
    )

    ekf_slam_node = Node(
        package='diff_drive_robot',
        executable='ekf_slam_node',
        name='ekf_slam_node',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'odom_frame': 'odom',
            'body_frame': 'base_link',
            'wheel_radius': 0.033,
            'wheel_separation': 0.170,
            'process_covariance': 0.01,
            'sensor_covariance': 0.001,
            'max_obstacles': 15,
            'dk_threshold': 5.991,
        }]
    )

    return LaunchDescription([
        LogInfo(msg=(
            'WARNING: ekf_slam.launch.py is an experimental landmark visualizer; '
            'it does not publish the OccupancyGrid required by Nav2. Use '
            'auto_slam.launch.py for autonomous mapping.'
        )),
        scan_topic_arg,
        landmarks_node,
        ekf_slam_node,
    ])
