from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare('robot_localization_config'),
        'config', 'ekf.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('odometry/filtered', '/odometry/filtered')]
        )
    ])
