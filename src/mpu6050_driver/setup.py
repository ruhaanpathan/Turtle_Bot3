import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mpu6050_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@robot.local',
    description='ROS 2 driver for MPU6050 IMU via I2C',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpu6050_node = mpu6050_driver.mpu6050_node:main',
        ],
    },
)
