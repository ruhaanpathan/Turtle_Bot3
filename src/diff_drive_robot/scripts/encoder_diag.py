#!/usr/bin/env python3
"""
Diagnostic script to test wheel ticks and odometry yaw accuracy.
Usage:
  ros2 run diff_drive_robot encoder_diag.py
  (or python3 encoder_diag.py)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class EncoderDiagNode(Node):
    def __init__(self):
        super().__init__('encoder_diag')
        self.create_subscription(JointState, '/wheel_states', self._joint_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.last_print = 0
        self.yaw_deg = 0.0
        self.pos_l = 0.0
        self.pos_r = 0.0

        self.get_logger().info('Encoder Diagnostic Node started.')
        self.get_logger().info('Rotate your robot physically by 360 degrees by hand to test yaw accuracy.')

    def _joint_cb(self, msg: JointState):
        if len(msg.position) >= 2:
            self.pos_l = msg.position[0]
            self.pos_r = msg.position[1]

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        # Log at ~2 Hz
        now = self.get_clock().now().nanoseconds
        if now - self.last_print > 500_000_000:
            self.last_print = now
            l_rev = self.pos_l / (2 * math.pi)
            r_rev = self.pos_r / (2 * math.pi)
            self.get_logger().info(
                f"[DIAG] Left: {l_rev:+.2f} revs | Right: {r_rev:+.2f} revs | Odom Yaw: {self.yaw_deg:+.1f}°"
            )

def main():
    rclpy.init()
    node = EncoderDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
