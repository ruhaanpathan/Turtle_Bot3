#!/bin/bash
echo "Stopping all robot nodes..."
pkill -f "ydlidar_ros2_driver" 2>/dev/null
pkill -f "mpu6050_node"        2>/dev/null
pkill -f "motor_driver_node"   2>/dev/null
pkill -f "ekf_node"            2>/dev/null
pkill -f "slam_toolbox"        2>/dev/null
pkill -f "nav2"                2>/dev/null
echo "All nodes stopped."
