#!/bin/bash
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source ~/ydlidar_ros2_ws/install/setup.bash && source ~/robot_ws/install/setup.bash"
DELAY=10                                        # ← increased from 6 to 10
MAP_PATH="$HOME/my_map_final"
NAV2_PARAMS="$HOME/robot_ws/src/slam_nav2_bringup/config/nav2_params.yaml"
SLAM_PARAMS="$HOME/robot_ws/src/slam_nav2_bringup/config/slam_params.yaml"
RVIZ_CONFIG="$HOME/robot_ws/src/slam_nav2_bringup/rviz/robot_viz.rviz"

# ── AUTO-FIX I2C PERMISSIONS ─────────────────────────────────────────
fix_i2c_permissions() {
    if [ -e /dev/i2c-1 ]; then
        # Check if user already has access
        if ! ls /dev/i2c-1 > /dev/null 2>&1; then
            echo -e "  ${YELLOW}⚠️  I2C permission issue detected. Fixing...${NC}"
            sudo chmod 666 /dev/i2c-1
            echo -e "  ${GREEN}✅ I2C permissions fixed for this session.${NC}"
            echo -e "  ${CYAN}   For permanent fix: sudo usermod -aG i2c \$USER && reboot${NC}"
        fi
        # Add udev rule if it doesn't exist
        if [ ! -f /etc/udev/rules.d/99-i2c.rules ]; then
            echo -e "  ${CYAN}   Installing permanent udev rule...${NC}"
            echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null
            sudo usermod -aG i2c "$USER" 2>/dev/null
            sudo udevadm control --reload-rules
            sudo udevadm trigger
            echo -e "  ${GREEN}✅ Permanent udev rule installed.${NC}"
        fi
    else
        echo -e "  ${RED}⚠️  /dev/i2c-1 not found. Check I2C is enabled (raspi-config).${NC}"
    fi
}

# ── VERIFY IMU IS REACHABLE ──────────────────────────────────────────
check_imu() {
    if command -v i2cdetect &>/dev/null; then
        ADDR=$(i2cdetect -y 1 2>/dev/null | grep -o "68\|69" | head -1)
        if [ -n "$ADDR" ]; then
            echo -e "  ${GREEN}✅ MPU6050 detected at I2C address 0x${ADDR}${NC}"
            return 0
        else
            echo -e "  ${RED}⚠️  MPU6050 not detected on I2C bus. Check wiring (SDA/SCL/VCC/GND).${NC}"
            return 1
        fi
    fi
}

# ── WAIT FOR TF FRAME ────────────────────────────────────────────────
wait_for_frame() {
    local FRAME=$1
    local TIMEOUT=${2:-30}
    local COUNT=0
    echo -ne "  ${CYAN}   Waiting for TF frame '${FRAME}'...${NC}"
    while [ $COUNT -lt $TIMEOUT ]; do
        bash -c "$SOURCE_CMD && ros2 run tf2_ros tf2_echo map odom 2>/dev/null" | grep -q "translation" && {
            echo -e " ${GREEN}✅${NC}"; return 0
        }
        sleep 1; COUNT=$((COUNT+1))
        echo -ne "."
    done
    echo -e " ${YELLOW}(timeout — continuing anyway)${NC}"
}

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

# ── PRE-FLIGHT CHECK ─────────────────────────────────────────────────
echo -e "${BOLD}  Pre-flight checks...${NC}"
fix_i2c_permissions
check_imu
echo ""

echo -e "${BOLD}  What do you want to do?${NC}"
echo ""
echo -e "  ${GREEN}[1]${NC} 🗺️   Generate Map        — drive robot to map environment"
echo -e "  ${BLUE}[2]${NC} 🤖  Run Autonomous Nav   — navigate on saved map"
echo -e "  ${CYAN}[3]${NC} 📊  Visualize in RViz2   — view map, scan, pose, costmap"
echo -e "  ${YELLOW}[5]${NC} 🔍  Diagnostics          — check all nodes, topics, TF tree"
echo -e "  ${RED}[4]${NC} ⛔  Stop All             — kill all running nodes"
echo ""
echo -n "  Enter choice [1/2/3/4/5]: "
read -r MODE

# ── HELPERS ──────────────────────────────────────────────────────────
stop_all() {
    echo -e "\n${RED}Stopping all robot nodes...${NC}"
    pkill -f "ydlidar_ros2_driver" 2>/dev/null
    pkill -f "mpu6050_node"        2>/dev/null
    pkill -f "motor_driver_node"   2>/dev/null
    pkill -f "ekf_node"            2>/dev/null
    pkill -f "ekf_filter_node"     2>/dev/null
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

    # ── IMU: try to launch, warn if it fails ────────────────────────
    echo -e "  ${GREEN}[2/5]${NC} Starting MPU6050 IMU..."
    if bash -c "$SOURCE_CMD && ros2 pkg list 2>/dev/null | grep -q 'mpu6050_imu'"; then
        launch_term "🔵 IMU MPU6050" \
            "echo 'Keep robot STILL for calibration...' && ros2 run mpu6050_imu mpu6050_node"
        echo -e "        Waiting ${DELAY}s for calibration..."; sleep $DELAY
        echo -e "  ${GREEN}✅ IMU node launched.${NC}"
    else
        echo -e "  ${YELLOW}⚠️  mpu6050_imu package not found in install. Trying rebuild...${NC}"
        bash -c "cd ~/robot_ws && colcon build --packages-select mpu6050_imu 2>&1 | tail -5"
        if bash -c "$SOURCE_CMD && ros2 pkg list 2>/dev/null | grep -q 'mpu6050_imu'"; then
            launch_term "🔵 IMU MPU6050" \
                "echo 'Keep robot STILL for calibration...' && ros2 run mpu6050_imu mpu6050_node"
            echo -e "        Waiting ${DELAY}s for calibration..."; sleep $DELAY
        else
            echo -e "  ${RED}⚠️  IMU node unavailable — continuing without IMU (odometry only).${NC}"
            echo -e "  ${RED}   Run: cd ~/robot_ws && colcon build --packages-select mpu6050_imu${NC}"
            sleep 3
        fi
    fi

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
    launch_term "🟢 SLAM Toolbox" \
        "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$SLAM_PARAMS use_sim_time:=false"
    echo -e "        Waiting ${DELAY}s..."; sleep $DELAY

    echo -e "  ${GREEN}[+]${NC}  Starting Teleop Keyboard..."
    launch_term "🎮 TELEOP" \
        "echo 'i=fwd  ,=back  j=left  l=right  k=stop  q/z=speed' && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

    echo -e "  ${GREEN}[+]${NC}  Starting RViz2 for live map preview..."
    launch_term "📊 RViz2 — Map Preview" "sleep 5 && ros2 run rviz2 rviz2 -d $RVIZ_CONFIG"  # ← sleep 3→5

    echo ""
    echo -e "${BOLD}${YELLOW}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║  All nodes running!                                  ║"
    echo "  ║                                                      ║"
    echo "  ║  📋 MAPPING TIPS:                                    ║"
    echo "  ║  • Drive SLOWLY for better scan quality              ║"
    echo "  ║  • Cover all rooms/corridors at least twice          ║"
    echo "  ║  • Return to start point to close the loop           ║"
    echo "  ║  • Straight walls should look straight in RViz       ║"
    echo "  ║                                                      ║"
    echo "  ║  👉 Use TELEOP terminal to drive the robot           ║"
    echo "  ║  👉 Watch the map build live in RViz2                ║"
    echo "  ║                                                      ║"
    echo "  ║  When environment is fully mapped,                   ║"
    echo "  ║  press ENTER here to save the map.                   ║"
    echo "  ╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -n "  Press ENTER to save map... "
    read -r

    echo -e "  ${CYAN}Saving map to ${MAP_PATH}...${NC}"
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
        echo -e "${RED}  ⚠️  Map save failed. Is SLAM Toolbox still running?${NC}"
    fi

# ── MODE 2: AUTONOMOUS NAVIGATION ────────────────────────────────────
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
    launch_term "🧭 NAV2 Navigation" \
        "ros2 launch nav2_bringup bringup_launch.py map:=${MAP_PATH}.yaml use_sim_time:=false params_file:=$NAV2_PARAMS"
    echo -e "        Waiting $((DELAY * 2))s for NAV2 to fully initialize..."; sleep $((DELAY * 2))  # ← double delay for NAV2

    echo -e "  ${GREEN}[+]${NC}  Starting RViz2..."
    launch_term "📊 RViz2 — Navigation" "sleep 5 && ros2 run rviz2 rviz2 -d $RVIZ_CONFIG"  # ← sleep 3→5

    echo ""
    echo -e "  ${CYAN}Checking TF tree...${NC}"
    TF_OK=$(bash -c "$SOURCE_CMD && ros2 topic echo /tf --once 2>/dev/null" | grep -c "frame_id" || true)
    if [ "$TF_OK" -gt 0 ]; then
        echo -e "  ${GREEN}✅ TF frames are publishing.${NC}"
    else
        echo -e "  ${YELLOW}⚠️  TF may not be ready yet — wait for RViz to open.${NC}"
    fi

    echo -e "${BOLD}${BLUE}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║   ✅  Autonomous navigation running!                 ║"
    echo "  ║                                                      ║"
    echo "  ║   STEP 1: In RViz2 click [2D Pose Estimate]         ║"
    echo "  ║           → click where robot is on the map         ║"
    echo "  ║           → drag in the direction robot faces       ║"
    echo "  ║           → wait for green particles to converge    ║"
    echo "  ║                                                      ║"
    echo "  ║   STEP 2: Click [Nav2 Goal] → click destination     ║"
    echo "  ║           → robot will plan path and drive!         ║"
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

# ── MODE 5: DIAGNOSTICS ──────────────────────────────────────────────
elif [ "$MODE" == "5" ]; then
    clear
    echo -e "${BOLD}${YELLOW}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║          DIAGNOSTICS MODE                ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"

    echo -e "${BOLD}  ── I2C / IMU ───────────────────────────────${NC}"
    fix_i2c_permissions
    check_imu

    echo ""
    echo -e "${BOLD}  ── Active ROS2 Nodes ───────────────────────${NC}"
    bash -c "$SOURCE_CMD && ros2 node list 2>/dev/null" | while read -r node; do
        echo -e "  ${GREEN}●${NC} $node"
    done

    echo ""
    echo -e "${BOLD}  ── Key Topics ──────────────────────────────${NC}"
    for TOPIC in /scan /odom /imu /map /odometry/filtered /cmd_vel; do
        EXISTS=$(bash -c "$SOURCE_CMD && ros2 topic list 2>/dev/null | grep -c '^${TOPIC}$'" 2>/dev/null || echo "0")
        if [ "$EXISTS" -gt 0 ]; then
            HZ=$(bash -c "$SOURCE_CMD && timeout 3 ros2 topic hz $TOPIC 2>/dev/null | grep 'average rate' | awk '{print \$3}'" 2>/dev/null || echo "?")
            echo -e "  ${GREEN}✅${NC} $TOPIC  (${HZ} Hz)"
        else
            echo -e "  ${RED}❌${NC} $TOPIC  — not publishing"
        fi
    done

    echo ""
    echo -e "${BOLD}  ── TF Tree ─────────────────────────────────${NC}"
    bash -c "$SOURCE_CMD && ros2 run tf2_tools view_frames 2>/dev/null" && \
        echo -e "  ${GREEN}✅ frames.pdf saved in current directory.${NC}" || \
        echo -e "  ${RED}❌ Could not generate TF tree.${NC}"

    TF_CHAIN=$(bash -c "$SOURCE_CMD && ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -3" 2>/dev/null)
    if echo "$TF_CHAIN" | grep -q "translation"; then
        echo -e "  ${GREEN}✅ Full chain:  map → odom → base_link → laser_frame  OK${NC}"
    else
        echo -e "  ${RED}❌ map → base_link chain broken. Give 2D Pose Estimate in RViz!${NC}"
    fi

    echo ""
    echo -e "${BOLD}  ── Summary ─────────────────────────────────${NC}"
    bash -c "$SOURCE_CMD && ros2 node list 2>/dev/null" > /tmp/nodelist.txt
    for NODE in ydlidar_ros2_driver mpu6050 motor_driver ekf_filter slam_toolbox amcl map_server; do
        if grep -qi "$NODE" /tmp/nodelist.txt; then
            echo -e "  ${GREEN}✅${NC} $NODE"
        else
            echo -e "  ${RED}❌${NC} $NODE — NOT running"
        fi
    done
    rm -f /tmp/nodelist.txt

    echo ""
else
    echo -e "${RED}  Invalid choice.${NC}"; exit 1
fi

echo ""
echo -e "  ${CYAN}Run ~/robot2.sh anytime to restart.${NC}"
echo ""
