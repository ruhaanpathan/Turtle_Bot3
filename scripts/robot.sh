#!/bin/bash
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source ~/ydlidar_ros2_ws/install/setup.bash && source ~/robot_ws/install/setup.bash"
DELAY=6
MAP_PATH="$HOME/my_map_final"
NAV2_PARAMS="$HOME/robot_ws/src/slam_nav2_bringup/config/nav2_params.yaml"
SLAM_PARAMS="$HOME/robot_ws/src/slam_nav2_bringup/config/slam_params.yaml"
RVIZ_CONFIG="$HOME/robot_ws/src/slam_nav2_bringup/rviz/robot_viz.rviz"

clear
echo -e "${BOLD}${CYAN}"
echo "  ██████╗  ██████╗ ██████╗  ██████╗ ████████╗"
echo "  ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝"
echo "  ██████╔╝██║   ██║██████╔╝██║   ██║   ██║   "
echo "  ██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   "
echo "  ██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   "
echo "  ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝   ╚═╝   "
echo -e "${NC}"
echo -e "${BOLD}  RPi 5 | YDLidar X2L | MPU6050 | L293N | NAV2${NC}"
echo -e "  ─────────────────────────────────────────────"
echo ""
echo -e "${BOLD}  What do you want to do?${NC}"
echo ""
echo -e "  ${GREEN}[1]${NC} 🗺️   Generate Map        — drive robot to map environment"
echo -e "  ${BLUE}[2]${NC} 🤖  Run Autonomous Nav   — navigate on saved map"
echo -e "  ${CYAN}[3]${NC} 📊  Visualize in RViz2   — view map, scan, pose, costmap"
echo -e "  ${RED}[4]${NC} ⛔  Stop All             — kill all running nodes"
echo ""
echo -n "  Enter choice [1/2/3/4]: "
read -r MODE

stop_all() {
    echo -e "\n${RED}Stopping all robot nodes...${NC}"
    pkill -f "ydlidar_ros2_driver" 2>/dev/null
    pkill -f "mpu6050_node"        2>/dev/null
    pkill -f "motor_driver_node"   2>/dev/null
    pkill -f "ekf_node"            2>/dev/null
    pkill -f "slam_toolbox"        2>/dev/null
    pkill -f "nav2"                2>/dev/null
    pkill -f "map_server"          2>/dev/null
    pkill -f "amcl"                2>/dev/null
    pkill -f "bt_navigator"        2>/dev/null
    pkill -f "teleop_twist"        2>/dev/null
    pkill -f "rviz2"               2>/dev/null
    echo -e "${GREEN}All nodes stopped.${NC}"
}

launch_term() {
    gnome-terminal --title="$1" -- bash -c "$SOURCE_CMD && $2; exec bash" &
}

launch_base_nodes() {
    echo ""
    echo -e "  ${GREEN}[1/5]${NC} Starting YDLidar X2L..."
    launch_term "🔴 YDLidar X2L" "ros2 launch ydlidar_ros2_driver ydlidar_launch.py"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY

    echo -e "  ${GREEN}[2/5]${NC} Starting MPU6050 IMU..."
    launch_term "🔵 IMU MPU6050" "echo 'Keep robot STILL for calibration...' && ros2 run mpu6050_imu mpu6050_node"
    echo -e "        Waiting ${DELAY}s for calibration..."; sleep $DELAY

    echo -e "  ${GREEN}[3/5]${NC} Starting L293N Motor Driver..."
    launch_term "🟠 Motor Driver" "ros2 run motor_driver motor_driver_node"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY

    echo -e "  ${GREEN}[4/5]${NC} Starting EKF Localization..."
    launch_term "🟣 EKF Localization" "ros2 launch robot_localization_config ekf.launch.py"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY
}

# ── MODE 1: MAP GENERATION ───────────────────────────────────────────
if [ "$MODE" == "1" ]; then
    clear
    echo -e "${BOLD}${GREEN}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║        MAP GENERATION MODE               ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
    stop_all 2>/dev/null; sleep 2
    launch_base_nodes

    echo -e "  ${GREEN}[5/5]${NC} Starting SLAM Toolbox..."
    launch_term "🟢 SLAM Toolbox" "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$SLAM_PARAMS use_sim_time:=false"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY

    echo -e "  ${GREEN}[+]${NC}  Starting Teleop Keyboard..."
    launch_term "🎮 TELEOP" "echo 'i=fwd  ,=back  j=left  l=right  k=stop  q/z=speed' && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

    echo -e "  ${GREEN}[+]${NC}  Starting RViz2 for live map preview..."
    launch_term "📊 RViz2 — Map Preview" "sleep 3 && ros2 run rviz2 rviz2 -d $RVIZ_CONFIG"

    echo ""
    echo -e "${BOLD}${YELLOW}"
    echo "  ╔══════════════════════════════════════════════════╗"
    echo "  ║  All nodes running!                              ║"
    echo "  ║                                                  ║"
    echo "  ║  👉 Use TELEOP terminal to drive the robot       ║"
    echo "  ║  👉 Watch the map build live in RViz2            ║"
    echo "  ║                                                  ║"
    echo "  ║  When environment is fully mapped,               ║"
    echo "  ║  press ENTER here to save the map.              ║"
    echo "  ╚══════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -n "  Press ENTER to save map... "
    read -r

    echo -e "  ${CYAN}Saving map...${NC}"
    bash -c "$SOURCE_CMD && ros2 run nav2_map_server map_saver_cli -f $MAP_PATH"

    if [ -f "${MAP_PATH}.yaml" ]; then
        echo -e "${BOLD}${GREEN}"
        echo "  ╔══════════════════════════════════════════╗"
        echo "  ║   ✅  Map saved successfully!            ║"
        echo "  ║   📄 ~/my_map_final.yaml                 ║"
        echo "  ║   🖼️  ~/my_map_final.pgm                  ║"
        echo "  ║   Run script again → choose [2] for nav  ║"
        echo "  ╚══════════════════════════════════════════╝"
        echo -e "${NC}"
    else
        echo -e "${RED}  ⚠️  Map save failed.${NC}"
    fi

# ── MODE 2: AUTONOMOUS NAVIGATION ───────────────────────────────────
elif [ "$MODE" == "2" ]; then
    clear
    echo -e "${BOLD}${BLUE}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║       AUTONOMOUS NAVIGATION MODE         ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"

    if [ ! -f "${MAP_PATH}.yaml" ]; then
        echo -e "${RED}  ⚠️  No map found at ${MAP_PATH}.yaml"
        echo -e "  Please run Mode [1] first.${NC}"; exit 1
    fi

    echo -e "  ${GREEN}✅ Map found:${NC} ${MAP_PATH}.yaml"
    stop_all 2>/dev/null; sleep 2
    launch_base_nodes

    echo -e "  ${GREEN}[5/5]${NC} Starting NAV2..."
    launch_term "🧭 NAV2 Navigation" "ros2 launch nav2_bringup bringup_launch.py map:=${MAP_PATH}.yaml use_sim_time:=false params_file:=$NAV2_PARAMS"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY

    echo -e "  ${GREEN}[+]${NC}  Starting RViz2..."
    launch_term "📊 RViz2 — Navigation" "sleep 3 && ros2 run rviz2 rviz2 -d $RVIZ_CONFIG"

    echo -e "${BOLD}${BLUE}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║   ✅  Autonomous navigation running!                 ║"
    echo "  ║                                                      ║"
    echo "  ║   In RViz2 toolbar click:                            ║"
    echo "  ║   [2D Pose Estimate] → click robot's start position  ║"
    echo "  ║   [Nav2 Goal]        → click destination on map      ║"
    echo "  ║                                                      ║"
    echo "  ║   Or send goal from terminal:                        ║"
    echo "  ║   ros2 topic pub --once /goal_pose \                 ║"
    echo "  ║     geometry_msgs/msg/PoseStamped \                  ║"
    echo "  ║     \"{header: {frame_id: 'map'},                    ║"
    echo "  ║       pose: {position: {x: 1.0, y: 0.5},            ║"
    echo "  ║       orientation: {w: 1.0}}}\"                      ║"
    echo "  ╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"

# ── MODE 3: RVIZ2 ONLY ───────────────────────────────────────────────
elif [ "$MODE" == "3" ]; then
    clear
    echo -e "${BOLD}${CYAN}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║         RVIZ2 VISUALIZATION MODE         ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  Launching RViz2 with full robot config..."
    echo ""
    echo -e "  ${BOLD}Displays loaded:${NC}"
    echo -e "  🗺️   Map            → /map"
    echo -e "  🔴  LaserScan      → /scan"
    echo -e "  🟠  Odometry       → /odometry/filtered"
    echo -e "  🟢  NAV2 Path      → /plan"
    echo -e "  🔵  TF Tree        → all frames"
    echo -e "  🟣  Local Costmap  → /local_costmap/costmap"
    echo -e "  ⚪  Global Costmap → /global_costmap/costmap"
    echo ""
    echo -e "  ${YELLOW}⚠️  Make sure robot nodes are already running!${NC}"
    echo ""
    bash -c "$SOURCE_CMD && ros2 run rviz2 rviz2 -d $RVIZ_CONFIG"

# ── MODE 4: STOP ALL ─────────────────────────────────────────────────
elif [ "$MODE" == "4" ]; then
    stop_all
else
    echo -e "${RED}  Invalid choice.${NC}"; exit 1
fi

echo ""
echo -e "  ${CYAN}Run ~/robot.sh anytime to restart.${NC}"
echo ""
