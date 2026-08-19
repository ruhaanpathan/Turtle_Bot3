"""Unit tests for MPU6050 sample conversion without requiring I2C hardware."""

import math
import struct
import unittest

from mpu6050_driver.mpu6050_node import (
    _ACCEL_SCALE_2G,
    _G_TO_MS2,
    _GYRO_SCALE_250,
    MPU6050Node,
)


class _FakeBus:
    def __init__(self, samples):
        self._samples = samples

    def read_i2c_block_data(self, _address, _register, _length):
        return list(struct.pack('>7h', *self._samples))


class MPU6050ConversionTests(unittest.TestCase):
    def _node_with_samples(self, samples):
        node = object.__new__(MPU6050Node)
        node._bus = _FakeBus(samples)
        node._address = 0x68
        return node

    def test_zero_sample_converts_to_zero(self):
        values = self._node_with_samples([0] * 7)._read_raw()
        self.assertEqual(values, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_accelerometer_and_gyro_scale_and_sign(self):
        samples = [
            int(_ACCEL_SCALE_2G),
            -int(_ACCEL_SCALE_2G),
            int(_ACCEL_SCALE_2G / 2),
            1234,  # temperature is intentionally ignored
            int(_GYRO_SCALE_250),
            -int(_GYRO_SCALE_250),
            int(_GYRO_SCALE_250 * 10),
        ]
        ax, ay, az, gx, gy, gz = self._node_with_samples(samples)._read_raw()

        self.assertAlmostEqual(ax, _G_TO_MS2)
        self.assertAlmostEqual(ay, -_G_TO_MS2)
        self.assertAlmostEqual(az, _G_TO_MS2 / 2.0)
        self.assertAlmostEqual(gx, math.pi / 180.0)
        self.assertAlmostEqual(gy, -math.pi / 180.0)
        self.assertAlmostEqual(gz, 10.0 * math.pi / 180.0)


if __name__ == '__main__':
    unittest.main()
