#!/usr/bin/env python3
"""
Live Odometry & Wheel Diagnostic Tool for Differential Drive Robot.
Monitors /wheel_states and /odom topics to verify:
 1. Left and Right wheel tick direction (both must be + when moving forward)
 2. Linear velocity v (must be + when moving forward)
 3. Angular velocity w (must be near 0 when moving straight)
 4. Odometry Pose (X should increase when moving forward)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class OdomDiagNode(Node):
    def __init__(self):
        super().__init__('odom_teleop_diag')
        self.wheel_sub = self.create_subscription(
            JointState, '/wheel_states', self.wheel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("============================================================")
        self.get_logger().info(" 🔍 ODOMETRY & WHEEL DIAGNOSTIC NODE STARTED")
        self.get_logger().info(" Drive your robot FORWARD using the joystick/phone.")
        self.get_logger().info(" Both Left & Right velocities MUST be POSITIVE (+).")
        self.get_logger().info(" Linear speed v MUST be POSITIVE (+), Angular w ≈ 0.")
        self.get_logger().info("============================================================")

    def wheel_callback(self, msg: JointState):
        if len(msg.position) >= 2 and len(msg.velocity) >= 2:
            left_pos = msg.position[0]
            right_pos = msg.position[1]
            left_vel = msg.velocity[0]
            right_vel = msg.velocity[1]

            left_dir = "➕ FORWARD" if left_vel > 0.05 else ("➖ REVERSE" if left_vel < -0.05 else "⏹️ STOPPED")
            right_dir = "➕ FORWARD" if right_vel > 0.05 else ("➖ REVERSE" if right_vel < -0.05 else "⏹️ STOPPED")

            print(f"[WHEELS] L_pos={left_pos:6.2f} rad ({left_dir:<10}) | R_pos={right_pos:6.2f} rad ({right_dir:<10}) | L_vel={left_vel:5.2f} R_vel={right_vel:5.2f} rad/s")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw)

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        print(f"  [ODOM] Pose X={x:6.3f} m  Y={y:6.3f} m  Yaw={yaw_deg:6.1f}° | Linear v={v:5.2f} m/s  Angular w={w:5.2f} rad/s")


def main():
    rclpy.init()
    node = OdomDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
