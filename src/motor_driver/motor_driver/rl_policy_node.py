#!/usr/bin/env python3
"""
ROS 2 node that runs the RL policy (ONNX) on the physical TurtleBot.

Subscribes to /imu (MPU6050) for angular velocity and yaw.
Runs the trained policy, converts wheel speed actions to cmd_vel,
and publishes for the motor driver.

Observation mapping (4-dim): [vx_est, fwd_dist, wz_imu, yaw_error]
Action mapping (2-dim):      [left_wheel_speed, right_wheel_speed]
  -> Converted to cmd_vel via diff-drive kinematics
"""

import math
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import SetBool


def quaternion_to_yaw(q):
    """Extract yaw angle from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RLPolicyNode(Node):

    def __init__(self):
        super().__init__('rl_policy_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '/home/ros/policy.onnx')
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_spacing', 0.16)
        self.declare_parameter('imu_timeout', 0.5)
        self.declare_parameter('enabled', True)

        model_path = self.get_parameter('model_path').value
        self.rate = self.get_parameter('inference_rate').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_spacing = self.get_parameter('wheel_spacing').value
        self.imu_timeout = self.get_parameter('imu_timeout').value
        self.enabled = self.get_parameter('enabled').value

        # LiDAR obstacle avoidance parameters
        self.declare_parameter('safe_dist', 1.0)
        self.declare_parameter('danger_dist', 0.4)
        self.safe_dist = self.get_parameter('safe_dist').value
        self.danger_dist = self.get_parameter('danger_dist').value

        # Max wheel angular speed = max_linear_vel / wheel_radius
        self.max_wheel_speed = self.max_lin / self.wheel_radius  # 5.0 rad/s

        # --- Load ONNX model ---
        self.get_logger().info(f'Loading policy from: {model_path}')
        opts = ort.SessionOptions()
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, sess_options=opts)
        self.get_logger().info('Policy loaded successfully ✅')

        # --- State ---
        self.latest_imu = None
        self.last_imu_time = None
        self.initial_yaw = None
        self.forward_distance = 0.0   # Estimated forward distance traveled
        self.last_linear_cmd = 0.0    # Last commanded linear velocity
        self.last_time = None         # For distance integration
        
        self.latest_scan = None       # Latest LaserScan message
        self.front_dist = float('inf') # Minimum distance in front cone

        # --- Publishers / Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # --- Enable/Disable service ---
        self.enable_srv = self.create_service(
            SetBool, '~/enable', self.enable_callback
        )

        # --- Inference timer ---
        self.timer = self.create_timer(1.0 / self.rate, self.run_policy)

        self.get_logger().info(
            f'RL Policy Node started 🚀 '
            f'(rate={self.rate}Hz, max_lin={self.max_lin}, max_ang={self.max_ang})'
        )
        self.get_logger().info(
            f'Diff-drive: wheel_radius={self.wheel_radius}, '
            f'wheel_spacing={self.wheel_spacing}, '
            f'max_wheel_speed={self.max_wheel_speed:.1f} rad/s'
        )

    def imu_callback(self, msg: Imu):
        """Store latest IMU data."""
        self.latest_imu = msg
        self.last_imu_time = self.get_clock().now()

        # Capture initial yaw on first message
        if self.initial_yaw is None:
            self.initial_yaw = quaternion_to_yaw(msg.orientation)
            self.last_time = self.get_clock().now()
            self.get_logger().info(
                f'Initial heading captured: {math.degrees(self.initial_yaw):.1f}°'
            )

    def scan_callback(self, msg: LaserScan):
        """Process latest LiDAR scan and compute front distance."""
        self.latest_scan = msg
        
        # Calculate minimum distance in a cone directly ahead (-20 to +20 degrees)
        # LaserScan usually starts at angle_min and increments by angle_increment
        
        # We find indices that corresponds to the front cone
        front_cone_half_angle = math.radians(20.0)
        
        ranges = np.array(msg.ranges)
        # Handle invalid ranges (Inf, NaN, 0.0)
        ranges = np.where(np.isnan(ranges) | (ranges == 0.0), float('inf'), ranges)
        
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Normalize angles to [-pi, pi]
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        
        # Mask points within the front cone
        front_mask = np.abs(angles) <= front_cone_half_angle
        
        if np.any(front_mask):
            self.front_dist = np.min(ranges[front_mask])
        else:
            self.front_dist = float('inf')

    def enable_callback(self, request, response):
        """Service to enable/disable the policy."""
        self.enabled = request.data
        if not self.enabled:
            self.stop_robot()
        response.success = True
        response.message = f'Policy {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def stop_robot(self):
        """Publish zero velocity."""
        try:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.last_linear_cmd = 0.0
        except Exception:
            pass

    def run_policy(self):
        """Main inference loop — called at inference_rate Hz."""
        if not self.enabled:
            return

        # Safety: check IMU timeout
        if self.latest_imu is None or self.last_imu_time is None:
            self.get_logger().warn(
                'Waiting for /imu data...', throttle_duration_sec=2.0
            )
            return

        elapsed = (self.get_clock().now() - self.last_imu_time).nanoseconds * 1e-9
        if elapsed > self.imu_timeout:
            self.get_logger().warn(
                f'IMU timeout ({elapsed:.2f}s) — stopping robot',
                throttle_duration_sec=2.0
            )
            self.stop_robot()
            return

        # --- Update forward distance estimate ---
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        # Integrate: distance += |last_linear_cmd| * dt
        self.forward_distance += abs(self.last_linear_cmd) * dt

        # --- Build observation vector ---
        imu = self.latest_imu

        # obs[0]: estimated forward velocity (use 0 since no encoder)
        vx_est = 0.0

        # obs[1]: estimated forward distance traveled
        fwd_dist = self.forward_distance

        # obs[2]: angular velocity from IMU
        wz = imu.angular_velocity.z

        # obs[3]: yaw heading error relative to initial heading
        current_yaw = quaternion_to_yaw(imu.orientation)
        yaw_error = current_yaw - self.initial_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Construct observation
        obs = np.array([[vx_est, fwd_dist, wz, yaw_error]], dtype=np.float32)

        # --- Run inference ---
        action = self.session.run(["action"], {"obs": obs})[0][0]

        # Actions are [left_wheel_speed, right_wheel_speed] in [-1, 1]
        # Scale to wheel angular velocities
        left_wheel = float(action[0]) * self.max_wheel_speed
        right_wheel = float(action[1]) * self.max_wheel_speed

        # Convert diff-drive wheel speeds to cmd_vel (linear, angular)
        linear_cmd = (left_wheel + right_wheel) * self.wheel_radius / 2.0
        angular_cmd = (right_wheel - left_wheel) * self.wheel_radius / self.wheel_spacing

        # Clamp to safe limits
        linear_cmd = max(-self.max_lin, min(self.max_lin, linear_cmd))
        angular_cmd = max(-self.max_ang, min(self.max_ang, angular_cmd))

        # --- Dynamic Obstacle Avoidance ---
        # Scale speed based on front distance
        if self.front_dist >= self.safe_dist:
            speed_mult = 1.0
        elif self.front_dist <= self.danger_dist:
            speed_mult = 0.0
        else:
            # Linear scaling between danger_dist and safe_dist
            speed_mult = (self.front_dist - self.danger_dist) / (self.safe_dist - self.danger_dist)
            
        linear_cmd *= speed_mult
        angular_cmd *= speed_mult

        # Store for distance integration
        self.last_linear_cmd = linear_cmd

        # --- Publish command ---
        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'obs=[{vx_est:.2f}, {fwd_dist:.3f}, {wz:.3f}, {yaw_error:.3f}] '
            f'front_dist={self.front_dist:.2f}m (mult={speed_mult:.2f}) '
            f'-> cmd_vel=[{linear_cmd:.3f}, {angular_cmd:.3f}]',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.stop_robot()
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
