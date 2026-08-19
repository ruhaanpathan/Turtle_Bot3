#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /home/ros/robot_ws/install/setup.bash 2>/dev/null || true
source /home/ros/ydlidar_ros2_ws/install/setup.bash 2>/dev/null || true

echo "🤖 Starting Fully Autonomous SLAM (Motors + IMU + YDLIDAR + SLAM Toolbox + Nav2 + Frontier Exploration)..."
ros2 launch diff_drive_robot auto_slam.launch.py
