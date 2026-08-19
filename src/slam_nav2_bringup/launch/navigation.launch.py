import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    diff_drive_pkg_dir = get_package_share_directory('diff_drive_robot')
    ydlidar_pkg_dir = get_package_share_directory('ydlidar_ros2_driver')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    slam_nav2_pkg_dir = get_package_share_directory('slam_nav2_bringup')
    ekf_pkg_dir = get_package_share_directory('robot_localization_config')

    # Default paths
    # NOTE: 'current_map.yaml' is a symlink maintained in /home/ros/maps/ that
    # always points at whichever map you want to navigate with right now.
    # Update the symlink (or pass map:=... explicitly) instead of editing this file
    # every time you re-map, so this default can never silently go stale again.
    default_map_path = '/home/ros/maps/current_map.yaml'
    default_nav2_params = os.path.join(slam_nav2_pkg_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(slam_nav2_pkg_dir, 'rviz', 'navigation.rviz')

    # Declare Launch Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load for navigation'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the ROS2 Nav2 parameters file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # 1. Start Motor Driver & Odometry Node
    diff_drive_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_drive_pkg_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Fuse wheel odometry and IMU and publish odom -> base_footprint.
    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_pkg_dir, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 2. Start YDLidar without a static TF; robot_state_publisher owns sensor TFs.
    ydlidar_params_file = os.path.join(ydlidar_pkg_dir, 'params', 'ydlidar.yaml')
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params_file],
        namespace='/',
        respawn=True,
        respawn_delay=2.0,
        respawn_max_retries=3,
    )

    # 3. Start map server + AMCL localization.
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'false',
        }.items()
    )

    # Launch only the Nav2 servers required for point-to-point navigation.
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_nav2_pkg_dir, 'launch', 'minimal_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nav2_params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': 'false',
        }.items()
    )

    # NOTE: We intentionally do NOT auto-publish /initialpose here anymore.
    # AMCL's map-frame origin is different for every map you save (it's wherever
    # the robot was physically standing when that SLAM session started), so a
    # blind "(0,0,0)" pose is only correct by coincidence. Set '2D Pose Estimate'
    # manually in RViz each run (see amcl.set_initial_pose: false in nav2_params.yaml).

    # 4. Start RViz2 with Nav2 configuration.
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # NOTE: mobile_teleop is intentionally NOT launched here anymore. Running the
    # web joystick at the same time as Nav2 let two different nodes publish to
    # /cmd_vel with no arbitration between them. Launch it separately
    # (ros2 launch mobile_teleop teleop.launch.py) only when you want manual
    # override, and stop it before/while trusting Nav2 to drive autonomously.

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)

    ld.add_action(diff_drive_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(ydlidar_node)
    ld.add_action(localization_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
