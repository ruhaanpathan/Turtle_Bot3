"""Basic packaging smoke test for mobile_teleop."""

import unittest

import mobile_teleop


class MobileTeleopPackageTests(unittest.TestCase):
    def test_package_imports(self):
        self.assertEqual(mobile_teleop.__name__, 'mobile_teleop')


if __name__ == '__main__':
    unittest.main()
