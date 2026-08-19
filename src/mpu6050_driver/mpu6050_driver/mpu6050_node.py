"""MPU6050 IMU ROS 2 driver node.

Reads accelerometer and gyroscope data from an MPU6050 via I2C (smbus2)
and publishes sensor_msgs/msg/Imu messages at a configurable rate.

Performs startup calibration by averaging readings at rest to determine
gyro/accel bias offsets for more accurate data.
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


# ── MPU6050 Register Addresses ──────────────────────────────────────────────
_PWR_MGMT_1 = 0x6B
_CONFIG = 0x1A
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B   # Start of 14-byte burst: accel(6) + temp(2) + gyro(6)
_WHO_AM_I = 0x75

# Scale factors (default ranges)
_ACCEL_SCALE_2G = 16384.0   # LSB/g for ±2g
_GYRO_SCALE_250 = 131.0     # LSB/(°/s) for ±250°/s
_DEG_TO_RAD = math.pi / 180.0
_G_TO_MS2 = 9.80665


class MPU6050Node(Node):
    """ROS 2 node that reads MPU6050 IMU data over I2C and publishes Imu messages."""

    def __init__(self):
        super().__init__('mpu6050_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)           # Hz
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_samples', 200)     # samples at startup
        self.declare_parameter('dlpf_mode', 3)                 # 0-6, higher = more smoothing

        # Calibration offsets (can be pre-set from yaml or computed at startup)
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)

        # Covariance (diagonal) — adjust based on your sensor quality
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.04)
        # Set true if turning left produces a negative Z angular velocity.
        self.declare_parameter('invert_gyro_z', True)
        self.declare_parameter('invert_gyro_x', False)
        self.declare_parameter('invert_gyro_y', False)

        self._bus_num = self.get_parameter('i2c_bus').value
        self._address = self.get_parameter('i2c_address').value
        self._rate = self.get_parameter('publish_rate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._cal_samples = self.get_parameter('calibration_samples').value
        self._dlpf = self.get_parameter('dlpf_mode').value

        self._gyro_offset = [
            self.get_parameter('gyro_offset_x').value,
            self.get_parameter('gyro_offset_y').value,
            self.get_parameter('gyro_offset_z').value,
        ]
        self._accel_offset = [
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value,
        ]
        self._gyro_cov = self.get_parameter('angular_velocity_covariance').value
        self._accel_cov = self.get_parameter('linear_acceleration_covariance').value
        self._invert_gz = self.get_parameter('invert_gyro_z').value
        self._invert_gx = self.get_parameter('invert_gyro_x').value
        self._invert_gy = self.get_parameter('invert_gyro_y').value

        self._validate_parameters()

        # ── I2C Initialisation ──────────────────────────────────────────────
        try:
            import smbus2
            self._bus = smbus2.SMBus(self._bus_num)
        except ImportError:
            self.get_logger().fatal(
                'smbus2 not installed. Run: pip install smbus2')
            raise SystemExit(1)
        except FileNotFoundError:
            self.get_logger().fatal(
                f'/dev/i2c-{self._bus_num} not found. '
                'Enable I2C: sudo raspi-config → Interface Options → I2C')
            raise SystemExit(1)

        try:
            self._init_sensor()

        # ── Calibration ─────────────────────────────────────────────────────
            # Only calibrate if offsets are all zero (not pre-configured).
            if all(v == 0.0 for v in self._gyro_offset + self._accel_offset):
                self._calibrate()
        except Exception:
            self._bus.close()
            raise

        # ── Publisher ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── Timer ───────────────────────────────────────────────────────────
        timer_period = 1.0 / self._rate
        self._timer = self.create_timer(timer_period, self._publish_imu)

        self.get_logger().info(
            f'MPU6050 ready — bus={self._bus_num} addr=0x{self._address:02X} '
            f'rate={self._rate}Hz dlpf={self._dlpf} '
            f'gyro_offset={[round(v, 4) for v in self._gyro_offset]} '
            f'accel_offset={[round(v, 4) for v in self._accel_offset]}')

    # ════════════════════════════════════════════════════════════════════════
    #  Sensor init
    # ════════════════════════════════════════════════════════════════════════
    def _validate_parameters(self):
        """Reject unsafe or nonsensical settings before opening the I2C device."""
        if not isinstance(self._bus_num, int) or self._bus_num < 0:
            raise ValueError('i2c_bus must be a non-negative integer')
        if not isinstance(self._address, int) or not 0 <= self._address <= 0x7F:
            raise ValueError('i2c_address must be a valid 7-bit I2C address')
        if not math.isfinite(float(self._rate)) or self._rate <= 0.0:
            raise ValueError('publish_rate must be finite and greater than zero')
        if not isinstance(self._cal_samples, int) or self._cal_samples < 10:
            raise ValueError('calibration_samples must be an integer of at least 10')
        if not isinstance(self._dlpf, int) or not 0 <= self._dlpf <= 6:
            raise ValueError('dlpf_mode must be an integer from 0 through 6')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')
        values = self._gyro_offset + self._accel_offset
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('calibration offsets must be finite')
        if (not math.isfinite(float(self._gyro_cov)) or self._gyro_cov < 0.0 or
                not math.isfinite(float(self._accel_cov)) or self._accel_cov < 0.0):
            raise ValueError('IMU covariance values must be finite and non-negative')

    def _init_sensor(self):
        """Wake up MPU6050 and configure DLPF + ranges."""
        # Verify WHO_AM_I
        who = self._bus.read_byte_data(self._address, _WHO_AM_I)
        if who not in (0x68, 0x98):  # 0x98 for some clones
            self.get_logger().warn(
                f'Unexpected WHO_AM_I: 0x{who:02X} (expected 0x68)')

        # Wake up: use gyro X PLL clock (more accurate than internal RC)
        self._bus.write_byte_data(self._address, _PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # DLPF: digital low-pass filter (reduces noise from vibrations)
        # Mode 3 → 44Hz bandwidth, 4.9ms delay — good balance for robotics
        self._bus.write_byte_data(self._address, _CONFIG, self._dlpf & 0x07)

        # Gyro range: ±250°/s (highest resolution for slow robot)
        self._bus.write_byte_data(self._address, _GYRO_CONFIG, 0x00)

        # Accel range: ±2g (highest resolution, sufficient for ground robot)
        self._bus.write_byte_data(self._address, _ACCEL_CONFIG, 0x00)

        time.sleep(0.05)
        self.get_logger().info('MPU6050 sensor initialised')

    # ════════════════════════════════════════════════════════════════════════
    #  Startup calibration
    # ════════════════════════════════════════════════════════════════════════
    def _calibrate(self):
        """Average N readings at rest to compute gyro and accel bias offsets."""
        self.get_logger().info(
            f'Calibrating MPU6050 — keep robot STILL for '
            f'{self._cal_samples} samples ...')

        gx_sum = gy_sum = gz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        valid = 0

        for _ in range(self._cal_samples):
            try:
                ax, ay, az, gx, gy, gz = self._read_raw()
                gx_sum += gx
                gy_sum += gy
                gz_sum += gz
                ax_sum += ax
                ay_sum += ay
                az_sum += az
                valid += 1
            except OSError:
                pass   # skip I2C errors during calibration
            time.sleep(0.005)  # ~200Hz read during calibration

        if valid < 10:
            self.get_logger().error(
                'Calibration failed — too few valid reads. Check wiring.')
            return

        # Gyro offset: average should be zero at rest
        self._gyro_offset[0] = gx_sum / valid
        self._gyro_offset[1] = gy_sum / valid
        self._gyro_offset[2] = gz_sum / valid

        # Accel offset: at rest, Z should read 1g; X and Y should be 0
        self._accel_offset[0] = ax_sum / valid
        self._accel_offset[1] = ay_sum / valid
        # Z offset is relative to 1g (gravity)
        self._accel_offset[2] = (az_sum / valid) - _G_TO_MS2

        self.get_logger().info(
            f'Calibration complete ({valid} samples) — '
            f'gyro_offset=[{self._gyro_offset[0]:.4f}, '
            f'{self._gyro_offset[1]:.4f}, {self._gyro_offset[2]:.4f}] '
            f'accel_offset=[{self._accel_offset[0]:.4f}, '
            f'{self._accel_offset[1]:.4f}, {self._accel_offset[2]:.4f}]')

    # ════════════════════════════════════════════════════════════════════════
    #  Raw I2C read
    # ════════════════════════════════════════════════════════════════════════
    def _read_raw(self):
        """Read and convert one accelerometer and gyroscope sample.

        Returns ``(ax, ay, az, gx, gy, gz)`` in m/s² and rad/s.
        """
        data = self._bus.read_i2c_block_data(self._address, _ACCEL_XOUT_H, 14)

        # Unpack as 7 big-endian signed 16-bit integers
        # Layout: accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z
        raw = struct.unpack('>7h', bytes(data))

        # Convert accelerometer (LSB → m/s²)
        ax = (raw[0] / _ACCEL_SCALE_2G) * _G_TO_MS2
        ay = (raw[1] / _ACCEL_SCALE_2G) * _G_TO_MS2
        az = (raw[2] / _ACCEL_SCALE_2G) * _G_TO_MS2

        # raw[3] = temperature (ignored)

        # Convert gyroscope (LSB → rad/s)
        gx = (raw[4] / _GYRO_SCALE_250) * _DEG_TO_RAD
        gy = (raw[5] / _GYRO_SCALE_250) * _DEG_TO_RAD
        gz = (raw[6] / _GYRO_SCALE_250) * _DEG_TO_RAD

        return ax, ay, az, gx, gy, gz

    # ════════════════════════════════════════════════════════════════════════
    #  Publisher callback
    # ════════════════════════════════════════════════════════════════════════
    def _publish_imu(self):
        """Read sensor, apply calibration, publish Imu message."""
        try:
            ax, ay, az, gx, gy, gz = self._read_raw()
        except OSError as e:
            self.get_logger().warn(f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply calibration offsets
        ax -= self._accel_offset[0]
        ay -= self._accel_offset[1]
        az -= self._accel_offset[2]
        gx -= self._gyro_offset[0]
        gy -= self._gyro_offset[1]
        gz -= self._gyro_offset[2]

        if self._invert_gx:
            gx = -gx
        if self._invert_gy:
            gy = -gy
        if self._invert_gz:
            gz = -gz

        # Build Imu message
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Orientation: NOT computed by MPU6050 (no onboard fusion)
        # Set covariance[0] = -1 to tell robot_localization "ignore orientation"
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0   # identity quaternion
        msg.orientation_covariance[0] = -1.0  # -1 = orientation not available

        # Angular velocity (rad/s)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = self._gyro_cov
        msg.angular_velocity_covariance[4] = self._gyro_cov
        msg.angular_velocity_covariance[8] = self._gyro_cov

        # Linear acceleration (m/s²)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = self._accel_cov
        msg.linear_acceleration_covariance[4] = self._accel_cov
        msg.linear_acceleration_covariance[8] = self._accel_cov

        self._pub.publish(msg)

    def destroy_node(self):
        """Clean up I2C bus on shutdown."""
        try:
            self._bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
