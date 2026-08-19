#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  RL POLICY DEPLOYMENT SCRIPT
#  Launches ONLY the nodes needed for straight-line RL policy:
#    YDLidar → MPU6050 IMU → RL Policy → Motor Driver
#  NO EKF needed.
# ═══════════════════════════════════════════════════════════

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source ~/ydlidar_ros2_ws/install/setup.bash && source ~/robot_ws/install/setup.bash"
MODEL_PATH="$HOME/policy.onnx"

clear
echo -e "${BOLD}${CYAN}"
echo "  ██████╗ ██╗         ██████╗  ██████╗ ██╗     ██╗ ██████╗██╗   ██╗"
echo "  ██╔══██╗██║         ██╔══██╗██╔═══██╗██║     ██║██╔════╝╚██╗ ██╔╝"
echo "  ██████╔╝██║         ██████╔╝██║   ██║██║     ██║██║      ╚████╔╝ "
echo "  ██╔══██╗██║         ██╔═══╝ ██║   ██║██║     ██║██║       ╚██╔╝  "
echo "  ██║  ██║███████╗    ██║     ╚██████╔╝███████╗██║╚██████╗   ██║   "
echo "  ╚═╝  ╚═╝╚══════╝    ╚═╝      ╚═════╝ ╚══════╝╚═╝ ╚═════╝   ╚═╝   "
echo -e "${NC}"
echo -e "${BOLD}  RPi 5 | Trained RL Policy | Move Straight${NC}"
echo -e "  ──────────────────────────────────────────────"
echo ""

# ── PRE-FLIGHT CHECKS ────────────────────────────────────────────────
echo -e "  ${BOLD}Pre-flight checks:${NC}"

if [ ! -f "$MODEL_PATH" ]; then
    echo -e "  ${RED}✗ Policy model not found: $MODEL_PATH${NC}"
    echo -e "  ${YELLOW}  Run: python3 ~/convert_to_onnx.py${NC}"
    exit 1
else
    SIZE=$(stat -c%s "$MODEL_PATH" 2>/dev/null || echo "?")
    echo -e "  ${GREEN}✓${NC} Policy model: $MODEL_PATH ($SIZE bytes)"
fi

if python3 -c "import onnxruntime" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} onnxruntime installed"
else
    echo -e "  ${RED}✗ onnxruntime not installed${NC}"
    exit 1
fi

# Check I2C for MPU6050
if i2cdetect -y 1 2>/dev/null | grep -q "68"; then
    echo -e "  ${GREEN}✓${NC} MPU6050 IMU detected on I2C bus 1 (0x68)"
else
    echo -e "  ${RED}✗ MPU6050 not detected on I2C bus 1${NC}"
    echo -e "  ${YELLOW}  Check wiring: SDA→GPIO2, SCL→GPIO3, VCC→3.3V, GND→GND${NC}"
    exit 1
fi

echo -e "  ${CYAN}ℹ${NC}  EKF not needed (policy uses IMU directly). LiDAR used for obstacle override."
echo ""
echo -e "${BOLD}  Select mode:${NC}"
echo ""
echo -e "  ${GREEN}[1]${NC} 🤖  Run RL Policy          — drive straight using trained policy"
echo -e "  ${BLUE}[2]${NC} 🧪  Test Policy Only        — policy node only (base nodes already up)"
echo -e "  ${RED}[3]${NC} ⛔  Stop All               — kill all running nodes"
echo ""
echo -n "  Enter choice [1/2/3]: "
read -r MODE

stop_all() {
    echo -e "\n${RED}Stopping all robot nodes...${NC}"
    pkill -f "ydlidar_ros2_driver" 2>/dev/null
    pkill -f "rl_policy_node"      2>/dev/null
    pkill -f "mpu6050_node"        2>/dev/null
    pkill -f "motor_driver_node"   2>/dev/null
    sleep 1
    echo -e "${GREEN}All nodes stopped.${NC}"
}

launch_term() {
    gnome-terminal --title="$1" -- bash -c "$SOURCE_CMD && $2; exec bash" &
}

# ── MODE 1: FULL RL POLICY LAUNCH ────────────────────────────────────
if [ "$MODE" == "1" ]; then
    clear
    echo -e "${BOLD}${GREEN}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║        RL POLICY DEPLOYMENT MODE          ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
    stop_all 2>/dev/null; sleep 2

    echo ""
    echo -e "  ${GREEN}[1/4]${NC} Starting YDLidar X2L..."
    launch_term "🔴 YDLidar X2L" "ros2 launch ydlidar_ros2_driver ydlidar_launch.py"
    echo -e "        Waiting 5s..."; sleep 5

    echo -e "  ${GREEN}[2/4]${NC} Starting MPU6050 IMU..."
    echo -e "        ${YELLOW}⚠  Keep robot STILL for calibration!${NC}"
    launch_term "🔵 IMU MPU6050" "ros2 run mpu6050_imu mpu6050_node"
    echo -e "        Waiting 8s for calibration..."; sleep 8

    echo -e "  ${GREEN}[3/4]${NC} Starting L293N Motor Driver..."
    launch_term "🟠 Motor Driver" "ros2 run motor_driver motor_driver_node"
    echo -e "        Waiting 4s..."; sleep 4

    echo -e "  ${GREEN}[4/4]${NC} Starting RL Policy Node (with LiDAR avoidance)..."
    launch_term "🧠 RL Policy" "ros2 run motor_driver rl_policy_node --ros-args -p model_path:=$MODEL_PATH -p inference_rate:=10.0 -p max_linear_vel:=0.5 -p max_angular_vel:=2.0"
    echo -e "        Waiting 3s..."; sleep 3

    echo ""
    echo -e "${BOLD}${GREEN}"
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║   ✅  RL Policy is RUNNING!                          ║"
    echo "  ║                                                      ║"
    echo "  ║   🧠 Policy: Move Straight (trained in Isaac Lab)    ║"
    echo "  ║   📊 Input:  [vx_cmd, 0, wz_imu, yaw_error]         ║"
    echo "  ║   🎯 Output: [linear_vel, angular_vel]               ║"
    echo "  ║   🔦 Avoid:  Slows down <1.0m, Stops <0.4m           ║"
    echo "  ║                                                      ║"
    echo "  ║   ⚡ 4 nodes: LiDAR + IMU + Motor + Policy           ║"
    echo "  ║                                                      ║"
    echo "  ║   Safety controls:                                   ║"
    echo "  ║   • Ctrl+C in policy terminal to stop               ║"
    echo "  ║   • ros2 service call /rl_policy_node/enable \\      ║"
    echo "  ║     std_srvs/srv/SetBool \"{data: false}\"            ║"
    echo "  ║                                                      ║"
    echo "  ║   Monitor:                                           ║"
    echo "  ║   • ros2 topic echo /cmd_vel                         ║"
    echo "  ║   • ros2 topic echo /imu                             ║"
    echo "  ╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo ""
    echo -e "  ${YELLOW}Press ENTER to stop all nodes...${NC}"
    read -r
    stop_all

# ── MODE 2: POLICY ONLY ──────────────────────────────────────────────
elif [ "$MODE" == "2" ]; then
    clear
    echo -e "${BOLD}${BLUE}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║        POLICY-ONLY TEST MODE              ║"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
    echo -e "  ${YELLOW}Make sure IMU & Motor Driver are already running!${NC}"
    echo ""
    echo -e "  Starting RL Policy Node..."
    bash -c "$SOURCE_CMD && ros2 run motor_driver rl_policy_node --ros-args -p model_path:=$MODEL_PATH -p inference_rate:=10.0 -p max_linear_vel:=0.5 -p max_angular_vel:=2.0"

# ── MODE 3: STOP ALL ─────────────────────────────────────────────────
elif [ "$MODE" == "3" ]; then
    stop_all

else
    echo -e "${RED}  Invalid choice.${NC}"; exit 1
fi

echo ""
echo -e "  ${CYAN}Run ~/run_policy.sh anytime to restart.${NC}"
echo ""
