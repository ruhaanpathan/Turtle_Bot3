#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  Robot Master Launch Script
#  Starts all nodes in separate terminals with 6s delays
# ═══════════════════════════════════════════════════════════

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

DELAY=6

echo "╔══════════════════════════════════════════╗"
echo "║       ROBOT STARTUP SEQUENCE             ║"
echo "╚══════════════════════════════════════════╝"

# ── Terminal 1: YDLidar ─────────────────────────────────────
echo "[1/5] Starting YDLidar X2L..."
gnome-terminal --title="YDLidar" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/ydlidar_ros2_ws/install/setup.bash
  echo '>>> YDLidar X2L starting...'
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py
  exec bash" &

echo "      Waiting ${DELAY}s for LiDAR to initialise..."
sleep $DELAY

# ── Terminal 2: MPU6050 IMU ─────────────────────────────────
echo "[2/5] Starting MPU6050 IMU..."
gnome-terminal --title="IMU MPU6050" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/robot_ws/install/setup.bash
  echo '>>> MPU6050 IMU starting — keep robot STILL for calibration...'
  ros2 run mpu6050_imu mpu6050_node
  exec bash" &

echo "      Waiting ${DELAY}s for IMU calibration..."
sleep $DELAY

# ── Terminal 3: Motor Driver ─────────────────────────────────
echo "[3/5] Starting L293N Motor Driver..."
gnome-terminal --title="Motor Driver" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/robot_ws/install/setup.bash
  echo '>>> L293N Motor Driver starting...'
  ros2 run motor_driver motor_driver_node
  exec bash" &

echo "      Waiting ${DELAY}s for motor driver..."
sleep $DELAY

# ── Terminal 4: EKF ──────────────────────────────────────────
echo "[4/5] Starting EKF (robot_localization)..."
gnome-terminal --title="EKF Localization" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/robot_ws/install/setup.bash
  echo '>>> EKF filter starting — fusing /odom + /imu...'
  ros2 launch robot_localization_config ekf.launch.py
  exec bash" &

echo "      Waiting ${DELAY}s for EKF to stabilise..."
sleep $DELAY

# ── Terminal 5: SLAM Toolbox ─────────────────────────────────
echo "[5/5] Starting SLAM Toolbox..."
gnome-terminal --title="SLAM Toolbox" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/robot_ws/install/setup.bash
  echo '>>> SLAM Toolbox starting — building map...'
  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$HOME/robot_ws/src/slam_nav2_bringup/config/slam_params.yaml \
    use_sim_time:=false
  exec bash" &

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║   All nodes launched successfully!       ║"
echo "║                                          ║"
echo "║   Topics to verify:                      ║"
echo "║   /scan          → 8 Hz   (LiDAR)        ║"
echo "║   /imu           → 50 Hz  (IMU)          ║"
echo "║   /odom          → 20 Hz  (Motors)       ║"
echo "║   /odometry/filtered → 30 Hz (EKF)       ║"
echo "║   /map           → active (SLAM)         ║"
echo "╚══════════════════════════════════════════╝"
echo ""
echo "To verify all topics:"
echo "  ros2 topic list"
echo ""
echo "To stop all nodes:"
echo "  ~/stop_robot.sh"

# ── Terminal 6: NAV2 ─────────────────────────────────────────
sleep 6
echo "[6/6] Starting NAV2..."
gnome-terminal --title="NAV2" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/robot_ws/install/setup.bash
  echo '>>> NAV2 starting...'
  ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:=$HOME/robot_ws/src/slam_nav2_bringup/config/nav2_params.yaml
  exec bash" &
