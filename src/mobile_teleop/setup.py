import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'mobile_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Web assets
        (os.path.join('share', package_name, 'web'),
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='WiFi-based mobile teleop with web joystick UI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_server = mobile_teleop.web_server:main',
        ],
    },
)
