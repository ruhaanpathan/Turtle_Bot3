"""
Launch file for mobile teleop.
Starts the self-contained web + WebSocket server node.
HTTP on port 8080 (joystick UI), WebSocket on port 9090 (command bridge).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Single node handles both HTTP serving and WebSocket /cmd_vel bridge
    web_server_cmd = Node(
        package='mobile_teleop',
        executable='web_server',
        name='teleop_web_server',
        output='screen',
        parameters=[{
            'http_port': 8080,
            'ws_port': 9090,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(web_server_cmd)

    return ld
