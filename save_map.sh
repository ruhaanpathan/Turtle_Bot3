#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /home/ros/robot_ws/install/setup.bash 2>/dev/null || true

MAPS_DIR="/home/ros/maps"
mkdir -p "$MAPS_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME="${1:-map_$TIMESTAMP}"
FULL_PATH="$MAPS_DIR/$MAP_NAME"

echo "💾 Saving map to: ${FULL_PATH}.yaml and ${FULL_PATH}.pgm ..."
ros2 run nav2_map_server map_saver_cli -f "$FULL_PATH"

# Automatically convert relative image path to absolute path in the YAML file
if [ -f "${FULL_PATH}.yaml" ]; then
    sed -i "s|^image: .*|image: ${FULL_PATH}.pgm|" "${FULL_PATH}.yaml"
fi

# Keep 'current_map.yaml' pointing at whichever map was just saved, so
# navigation.launch.py's default (and start_navigation.sh) never go stale.
ln -sf "${FULL_PATH}.yaml" "${MAPS_DIR}/current_map.yaml"
echo "🔗 current_map.yaml -> ${FULL_PATH}.yaml"

echo "✅ Map saved successfully with absolute path: ${FULL_PATH}.yaml"
