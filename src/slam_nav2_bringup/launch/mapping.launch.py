import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav2_bringup')
    diff_drive_dir = get_package_share_directory('diff_drive_robot')
    ekf_dir = get_package_share_directory('robot_localization_config')
    mobile_teleop_dir = get_package_share_directory('mobile_teleop')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the mapping display'),
        DeclareLaunchArgument(
            'use_teleop', default_value='true',
            description='Start the web teleoperation interface'),
        DeclareLaunchArgument(
            'ydlidar_params_file',
            default_value=os.path.join(ydlidar_dir, 'params', 'ydlidar.yaml'),
            description='YDLidar parameter file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'slam_params.yaml'),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(diff_drive_dir, 'rviz', 'mapping.rviz'),
            description='RViz mapping configuration'),
    ]

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_drive_dir, 'launch', 'robot.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_dir, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params_file, {'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=2.0,
        respawn_max_retries=3,
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    teleop = IncludeLaunchDescription(
        condition=IfCondition(use_teleop),
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(mobile_teleop_dir, 'launch', 'teleop.launch.py')),
    )

    return LaunchDescription(declared_arguments + [
        robot,
        ekf,
        ydlidar,
        slam,
        rviz,
        teleop,
    ])
