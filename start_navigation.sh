#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /home/ros/robot_ws/install/setup.bash 2>/dev/null || true
source /home/ros/ydlidar_ros2_ws/install/setup.bash 2>/dev/null || true

# Default to the 'current_map.yaml' symlink, which always points at the map
# you most recently saved with save_map.sh. Pass a path explicitly to override.
INPUT_MAP="${1:-/home/ros/maps/current_map.yaml}"

# Fix missing leading slash if user typed "home/ros/maps/..."
if [[ "$INPUT_MAP" != /* ]]; then
    INPUT_MAP="/$INPUT_MAP"
fi

MAP_ARG="$INPUT_MAP"

echo "🚀 Starting Autonomous Navigation Stack..."
echo "📍 Map path: $MAP_ARG"
echo "🎯 Use '2D Pose Estimate' in RViz2 to localize, then '2D Nav Goal' to navigate!"

ros2 launch slam_nav2_bringup navigation.launch.py map:="$MAP_ARG"
