"""Launch only the Nav2 servers needed for point-to-point exploration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build a resource-conscious Nav2 stack for the Raspberry Pi robot."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('nav2_params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    common = {
        'output': 'screen',
        'respawn': use_respawn,
        'respawn_delay': 2.0,
        'parameters': [configured_params],
        'arguments': ['--ros-args', '--log-level', log_level],
    }
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    command_remappings = tf_remappings + [('cmd_vel', 'cmd_vel_nav')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    navigation_nodes = GroupAction(actions=[
        SetParameter('use_sim_time', use_sim_time),
        Node(
            package='nav2_controller',
            executable='controller_server',
            remappings=command_remappings,
            **common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            remappings=tf_remappings,
            **common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=command_remappings,
            **common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=command_remappings,
            **common,
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            remappings=tf_remappings,
            **common,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            remappings=tf_remappings,
            **common,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            description='Full path to the Nav2 parameter file',
        ),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        navigation_nodes,
    ])
