#!/usr/bin/env python3
"""Compare wheel, IMU, and filtered yaw without commanding the robot.

Run this while slowly turning the robot by hand. Under ROS REP-103, a
counter-clockwise turn must be positive on every yaw-rate topic.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class YawDiagnostic(Node):
    """Report yaw-rate sign and scale disagreements between sensors."""

    def __init__(self):
        super().__init__('yaw_diagnostic')
        self._wheel_w = None
        self._imu_w = None
        self._filtered_w = None
        self._wheel_yaw = None
        self._filtered_yaw = None
        self._mismatch_samples = 0

        self.create_subscription(Odometry, '/odom', self._wheel_callback, 20)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_callback, 20)
        self.create_subscription(Imu, '/imu/data', self._imu_callback, 20)
        self.create_timer(0.5, self._report)

        self.get_logger().info(
            'Slowly turn the robot COUNTER-CLOCKWISE by hand; wheel, IMU, '
            'and filtered angular.z must all be positive.')
        self.get_logger().info(
            'This node is read-only and never publishes a velocity command.')

    @staticmethod
    def _yaw(message):
        q = message.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _wheel_callback(self, message):
        self._wheel_w = message.twist.twist.angular.z
        self._wheel_yaw = self._yaw(message)

    def _filtered_callback(self, message):
        self._filtered_w = message.twist.twist.angular.z
        self._filtered_yaw = self._yaw(message)

    def _imu_callback(self, message):
        self._imu_w = message.angular_velocity.z

    @staticmethod
    def _number(value):
        return 'waiting' if value is None else f'{value:+.3f}'

    def _report(self):
        self.get_logger().info(
            'angular.z rad/s: wheel=%s imu=%s filtered=%s | yaw deg: '
            'wheel=%s filtered=%s' % (
                self._number(self._wheel_w),
                self._number(self._imu_w),
                self._number(self._filtered_w),
                self._number(
                    None if self._wheel_yaw is None else
                    math.degrees(self._wheel_yaw)),
                self._number(
                    None if self._filtered_yaw is None else
                    math.degrees(self._filtered_yaw))))

        if self._wheel_w is None or self._imu_w is None:
            return

        moving_threshold = 0.12
        if (abs(self._wheel_w) < moving_threshold or
                abs(self._imu_w) < moving_threshold):
            self._mismatch_samples = 0
            return

        if self._wheel_w * self._imu_w < 0.0:
            self._mismatch_samples += 1
            if self._mismatch_samples >= 3:
                self.get_logger().error(
                    'YAW SIGN MISMATCH: wheel and IMU report opposite turns. '
                    'Do not fuse the IMU until its mounting/sign is corrected.')
            return

        self._mismatch_samples = 0
        ratio = abs(self._imu_w / self._wheel_w)
        if ratio < 0.5 or ratio > 2.0:
            self.get_logger().warn(
                f'Yaw scale disagreement: |imu/wheel|={ratio:.2f}; '
                'check wheel separation and IMU scale.')


def main(args=None):
    rclpy.init(args=args)
    node = YawDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
