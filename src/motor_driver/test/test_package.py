"""Basic packaging smoke test for the legacy motor_driver package."""

import unittest

import motor_driver


class MotorDriverPackageTests(unittest.TestCase):
    def test_package_imports(self):
        self.assertEqual(motor_driver.__name__, 'motor_driver')


if __name__ == '__main__':
    unittest.main()
