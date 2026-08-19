import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('slam_nav2_bringup')
    diff_drive_dir = get_package_share_directory('diff_drive_robot')
    ekf_dir = get_package_share_directory('robot_localization_config')
    frontier_dir = get_package_share_directory('frontier_exploration_ros2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    start_exploration = LaunchConfiguration('start_exploration')
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    frontier_params_file = LaunchConfiguration('frontier_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the mapping display'),
        DeclareLaunchArgument(
            'start_exploration', default_value='true',
            description='Start frontier exploration after the stack is ready'),
        DeclareLaunchArgument(
            'exploration_delay', default_value='2.0',
            description='Settling delay after all exploration dependencies are ready'),
        DeclareLaunchArgument(
            'save_map_on_completion', default_value='true',
            description='Save the map when frontier exploration completes'),
        DeclareLaunchArgument(
            'map_directory', default_value='/home/ros/maps',
            description='Absolute directory for timestamped map output'),
        DeclareLaunchArgument(
            'map_prefix', default_value='turtlebot_map',
            description='Safe filename prefix for saved maps'),
        DeclareLaunchArgument(
            'serialize_pose_graph', default_value='true',
            description='Also save the editable SLAM Toolbox pose graph'),
        DeclareLaunchArgument(
            'ydlidar_params_file',
            default_value=os.path.join(ydlidar_dir, 'params', 'ydlidar.yaml'),
            description='YDLidar parameter file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'slam_params.yaml'),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'frontier_params_file',
            default_value=os.path.join(diff_drive_dir, 'config', 'frontier_params.yaml'),
            description='Frontier explorer parameter file'),
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

    # robot_state_publisher is the sole owner of the laser static transform.
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

    # SLAM Toolbox is the only map -> odom transform publisher in this stack.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'minimal_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params_file,
            'autostart': 'true',
            'use_respawn': 'false',
        }.items(),
    )

    exploration = GroupAction(
        condition=IfCondition(start_exploration),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        frontier_dir, 'launch', 'frontier_explorer.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': frontier_params_file,
                    'autostart': 'false',
                }.items(),
            ),
            Node(
                package='slam_nav2_bringup',
                executable='exploration_readiness_gate.py',
                name='exploration_readiness_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'post_ready_delay': LaunchConfiguration('exploration_delay'),
                }],
            ),
            Node(
                package='slam_nav2_bringup',
                executable='auto_map_saver.py',
                name='auto_map_saver',
                output='screen',
                condition=IfCondition(
                    LaunchConfiguration('save_map_on_completion')),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_directory': LaunchConfiguration('map_directory'),
                    'map_prefix': LaunchConfiguration('map_prefix'),
                    'serialize_pose_graph': ParameterValue(
                        LaunchConfiguration('serialize_pose_graph'),
                        value_type=bool),
                }],
            ),
        ],
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

    return LaunchDescription(declared_arguments + [
        robot,
        ekf,
        ydlidar,
        slam,
        navigation,
        exploration,
        rviz,
    ])
