#!/usr/bin/env bash
# ==============================================================================
# CogProj Master Startup & Lifecycle Manager
#
# Primary single-script entry point for:
# - Prerequisite validation & dependency checking
# - Automatic workspace build management
# - Non-invasive driver discovery & reuse (camera, LiDAR, SLAM)
# - Real ONNX ore detection, tracking & counting pipeline
# - Remote Operator Console web server & Foxglove Bridge
# - Safe motion gating (starts STOPPED with deadman supervisor)
# - Graceful, non-destructive Ctrl+C / --stop lifecycle shutdown
#
# Usage:
#   ./scripts/run_cogproj.sh                 # Default full system (real detector, stopped)
#   ./scripts/run_cogproj.sh --mode manual   # Prepares manual teleop (stopped at startup)
#   ./scripts/run_cogproj.sh --mode auto     # Prepares autonomous navigation
#   ./scripts/run_cogproj.sh --mode mock     # Synthetic detector demonstration
#   ./scripts/run_cogproj.sh --install       # Install missing dependencies
#   ./scripts/run_cogproj.sh --status        # Inspect running components
#   ./scripts/run_cogproj.sh --stop          # Stop processes launched by this script
# ==============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$WS_DIR/log"
PID_FILE="$LOG_DIR/cogproj_runtime.pids"

mkdir -p "$LOG_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

STARTUP_MODE="real"
ACTION="start"

# ------------------------------------------------------------------------------
# Argument Parsing
# ------------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --install)
            ACTION="install"
            shift
            ;;
        --status)
            ACTION="status"
            shift
            ;;
        --stop)
            ACTION="stop"
            shift
            ;;
        --mode)
            case "$2" in
                real|default) STARTUP_MODE="real" ;;
                manual)       STARTUP_MODE="manual" ;;
                autonomous|auto) STARTUP_MODE="autonomous" ;;
                mock)         STARTUP_MODE="mock" ;;
                *)
                    echo -e "${RED}[!] Unknown mode: $2${NC} (Valid: real, manual, autonomous, mock)"
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--install] [--status] [--stop] [--mode <real|manual|autonomous|mock>]"
            exit 0
            ;;
        *)
            echo -e "${RED}[!] Unknown argument: $1${NC}"
            exit 1
            ;;
    esac
done

kill_tree() {
    local pid="$1"
    local sig="${2:-SIGINT}"
    local children
    children=$(pgrep -P "$pid" 2>/dev/null || true)
    for c in $children; do
        kill_tree "$c" "$sig"
    done
    kill -"$sig" "$pid" 2>/dev/null || true
}

# ------------------------------------------------------------------------------
# Action: Stop
# ------------------------------------------------------------------------------
if [[ "$ACTION" == "stop" ]]; then
    echo -e "${BLUE}${BOLD}[CogProj] Stopping managed runtime processes...${NC}"
    if [[ -f "$PID_FILE" ]]; then
        # Read tracked PIDs in reverse order
        mapfile -t PIDS < "$PID_FILE"
        for ((idx=${#PIDS[@]}-1; idx>=0; idx--)); do
            PID="${PIDS[idx]}"
            if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
                NAME=$(ps -p "$PID" -o comm= 2>/dev/null || echo "PID $PID")
                echo -e "  • Stopping $NAME ($PID) and children..."
                kill_tree "$PID" SIGINT
            fi
        done
        sleep 2
        # Force kill any lingering tracked processes
        for PID in "${PIDS[@]}"; do
            if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
                kill_tree "$PID" SIGKILL
            fi
        done
        rm -f "$PID_FILE"
        echo -e "${GREEN}[✓] Managed runtime processes stopped cleanly.${NC}"
    else
        echo -e "${YELLOW}[i] No active runtime PID file found at $PID_FILE.${NC}"
    fi
    exit 0
fi

# ------------------------------------------------------------------------------
# Action: Status
# ------------------------------------------------------------------------------
if [[ "$ACTION" == "status" ]]; then
    echo -e "${BLUE}${BOLD}======================================================================${NC}"
    echo -e "${BLUE}${BOLD}               CogProj System Runtime Status Inspection               ${NC}"
    echo -e "${BLUE}${BOLD}======================================================================${NC}"
    "$SCRIPT_DIR/hardware_diagnostics.sh"
    exit 0
fi

# ------------------------------------------------------------------------------
# Action: Install
# ------------------------------------------------------------------------------
if [[ "$ACTION" == "install" ]]; then
    echo -e "${BLUE}${BOLD}[CogProj] Running reproducible dependency installer...${NC}"
    "$SCRIPT_DIR/install_dependencies.sh"
    echo -e "${BLUE}${BOLD}[CogProj] Rebuilding workspace...${NC}"
    "$SCRIPT_DIR/build_workspace.sh"
    echo -e "${GREEN}[✓] Installation and workspace build complete.${NC}"
    exit 0
fi

# ------------------------------------------------------------------------------
# Action: Start Complete Robot System
# ------------------------------------------------------------------------------
echo -e "${BLUE}${BOLD}======================================================================${NC}"
echo -e "${BLUE}${BOLD}   CogProj Autonomous Rover System - Coordinated Startup              ${NC}"
echo -e "${BLUE}${BOLD}======================================================================${NC}"

# 1. Environment Configuration
echo -e "\n${CYAN}${BOLD}[1/7] Sourcing Platform Environment & Overlays...${NC}"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/setup_environment.sh"

# 2. Dependency Audit
echo -e "\n${CYAN}${BOLD}[2/7] Checking Required Dependencies...${NC}"
MISSING_DEPS=0
if ! python3 -c "import tornado" 2>/dev/null; then
    echo -e "${RED}[✗] Missing python3-tornado. Run: $0 --install${NC}"
    MISSING_DEPS=1
fi
if ! python3 -c "import cv2; assert hasattr(cv2, 'dnn')" 2>/dev/null; then
    echo -e "${RED}[✗] Missing python3-opencv with DNN support. Run: $0 --install${NC}"
    MISSING_DEPS=1
fi
if ! python3 -c "import numpy" 2>/dev/null; then
    echo -e "${RED}[✗] Missing python3-numpy. Run: $0 --install${NC}"
    MISSING_DEPS=1
fi

if [[ "$MISSING_DEPS" -ne 0 ]]; then
    echo -e "${RED}[!] Critical dependencies missing. Aborting startup.${NC}"
    exit 1
fi
echo -e "${GREEN}[✓] Dependencies verified.${NC}"

# 3. Model Weights Verification
echo -e "\n${CYAN}${BOLD}[3/7] Verifying Trained AI Model Weights...${NC}"
ONNX_MODEL="$WS_DIR/Rover (2)/best.onnx"
if [[ "$STARTUP_MODE" != "mock" ]]; then
    if [[ ! -f "$ONNX_MODEL" ]]; then
        echo -e "${RED}[✗] ONNX model weights not found at: $ONNX_MODEL${NC}"
        echo -e "    Use --mode mock for synthetic demonstration, or provide best.onnx."
        exit 1
    fi
    SIZE_MB=$(du -m "$ONNX_MODEL" | cut -f1)
    echo -e "${GREEN}[✓] ONNX model ready: $ONNX_MODEL (${SIZE_MB} MB)${NC}"
else
    echo -e "${YELLOW}[i] Running in MOCK detector mode (synthetic detections).${NC}"
fi

# 4. Check & Rebuild Workspace if needed
echo -e "\n${CYAN}${BOLD}[4/7] Checking Workspace Build Artifacts...${NC}"
if [[ ! -f "$WS_DIR/install/cogproj_dashboard/lib/cogproj_dashboard/dashboard_node" ]]; then
    echo -e "${YELLOW}[i] cogproj_dashboard build artifacts missing. Building workspace...${NC}"
    "$SCRIPT_DIR/build_workspace.sh"
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
fi
echo -e "${GREEN}[✓] Workspace overlay verified.${NC}"

# Initialize PID tracking
: > "$PID_FILE"

track_pid() {
    local pid="$1"
    echo "$pid" >> "$PID_FILE"
}

cleanup() {
    echo -e "\n${YELLOW}${BOLD}[CogProj] Shutting down managed components...${NC}"
    if [[ -f "$PID_FILE" ]]; then
        mapfile -t PIDS < "$PID_FILE"
        for ((idx=${#PIDS[@]}-1; idx>=0; idx--)); do
            PID="${PIDS[idx]}"
            if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
                kill_tree "$PID" SIGINT
            fi
        done
        sleep 2
        for PID in "${PIDS[@]}"; do
            if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
                kill_tree "$PID" SIGKILL
            fi
        done
        rm -f "$PID_FILE"
    fi
    echo -e "${GREEN}[✓] Clean shutdown complete. Pre-existing processes preserved.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 5. Non-Invasive Hardware Sensor Drivers & Transforms
echo -e "\n${CYAN}${BOLD}[5/7] Auditing & Launching Hardware Sensor Drivers...${NC}"

ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null || true)

# 5a. Physical Camera Driver
CAM_PUBS=$(ros2 topic info /camera/image_raw 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
if [[ -z "$CAM_PUBS" ]]; then CAM_PUBS=0; fi

if [[ "$CAM_PUBS" -gt 0 ]]; then
    echo -e "  • Camera Stream  : ${GREEN}[REUSING]${NC} Already active on /camera/image_raw ($CAM_PUBS publishers)"
else
    echo -e "  • Camera Stream  : ${BLUE}[STARTING]${NC} Launching headless camera_ros camera_node..."
    ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 > "$LOG_DIR/camera.log" 2>&1 &
    CAM_PID=$!
    track_pid "$CAM_PID"
    sleep 2
    echo -e "    ${GREEN}[✓] camera_node started (PID $CAM_PID)${NC}"
fi

# 5b. Physical RPLiDAR Driver
LIDAR_PUBS=$(ros2 topic info /scan 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
if [[ -z "$LIDAR_PUBS" ]]; then LIDAR_PUBS=0; fi

if [[ "$LIDAR_PUBS" -gt 0 ]]; then
    echo -e "  • LiDAR Stream   : ${GREEN}[REUSING]${NC} Already active on /scan ($LIDAR_PUBS publishers)"
else
    if [[ -e /dev/ttyUSB0 || -e /dev/lyra_lidar ]]; then
        echo -e "  • LiDAR Stream   : ${BLUE}[STARTING]${NC} Launching sllidar_node on /dev/lyra_lidar..."
        ros2 run sllidar_ros2 sllidar_node --ros-args \
            -p serial_port:=/dev/lyra_lidar \
            -p serial_baudrate:=460800 \
            -p frame_id:=lidar_link > "$LOG_DIR/lidar.log" 2>&1 &
        LIDAR_PID=$!
        track_pid "$LIDAR_PID"
        sleep 2
        echo -e "    ${GREEN}[✓] sllidar_node started (PID $LIDAR_PID)${NC}"
    else
        echo -e "  • LiDAR Stream   : ${YELLOW}[STANDBY]${NC} No serial LiDAR detected on /dev/ttyUSB0"
    fi
fi

# 5c. Robot URDF State Publisher (TF Tree)
if ! pgrep -f "robot_state_publisher" >/dev/null 2>&1; then
    URDF_FILE="/home/veerobot/lyra_ws/src/beetlebot_description/urdf/beetlebot.urdf"
    if [[ -f "$URDF_FILE" ]]; then
        echo -e "  • Robot URDF TF  : ${BLUE}[STARTING]${NC} Publishing BeetleBot TF kinematics..."
        ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" > "$LOG_DIR/robot_state_publisher.log" 2>&1 &
        RSP_PID=$!
        track_pid "$RSP_PID"
        echo -e "    ${GREEN}[✓] robot_state_publisher started (PID $RSP_PID)${NC}"
    fi
else
    echo -e "  • Robot URDF TF  : ${GREEN}[REUSING]${NC} robot_state_publisher already active"
fi

# 5d. Safety Command Gate
if ! pgrep -f "cmd_vel_gate" >/dev/null 2>&1; then
    echo -e "  • Safety Gate    : ${BLUE}[STARTING]${NC} Launching lyra_cmd_vel_gate..."
    ros2 run lyra_cmd_vel_gate cmd_vel_gate > "$LOG_DIR/cmd_vel_gate.log" 2>&1 &
    GATE_PID=$!
    track_pid "$GATE_PID"
    echo -e "    ${GREEN}[✓] cmd_vel_gate active (PID $GATE_PID)${NC}"
else
    echo -e "  • Safety Gate    : ${GREEN}[REUSING]${NC} cmd_vel_gate already active"
fi

# 5e. Real SLAM Toolbox
MAP_PUBS=$(ros2 topic info /map 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
if [[ -z "$MAP_PUBS" ]]; then MAP_PUBS=0; fi

if [[ "$MAP_PUBS" -gt 0 ]] || pgrep -f "slam_toolbox" >/dev/null 2>&1; then
    echo -e "  • SLAM Mapping   : ${GREEN}[REUSING]${NC} Existing SLAM Toolbox instance publishing /map"
else
    SLAM_CONF="/home/veerobot/lyra_ws/src/lyra_slam/config/slam_toolbox.yaml"
    if [[ -f "$SLAM_CONF" ]]; then
        echo -e "  • SLAM Mapping   : ${BLUE}[STARTING]${NC} Launching async_slam_toolbox_node..."
        ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
            --params-file "$SLAM_CONF" \
            -r scan:=/scan \
            -r odom:=/odometry/filtered > "$LOG_DIR/slam_toolbox.log" 2>&1 &
        SLAM_PID=$!
        track_pid "$SLAM_PID"
        echo -e "    ${GREEN}[✓] slam_toolbox started (PID $SLAM_PID)${NC}"
    fi
fi

# 6. CogProj Perception Pipeline
echo -e "\n${CYAN}${BOLD}[6/7] Starting CogProj Perception Pipeline...${NC}"
DETECTOR_ARG="real"
if [[ "$STARTUP_MODE" == "mock" ]]; then
    DETECTOR_ARG="mock"
fi

ros2 launch cogproj_bringup demo_showcase.launch.py \
    detector_mode:="$DETECTOR_ARG" \
    start_slam:=false \
    start_rviz:=false > "$LOG_DIR/perception.log" 2>&1 &
PERC_PID=$!
track_pid "$PERC_PID"
sleep 2
echo -e "${GREEN}[✓] Perception pipeline active (PID $PERC_PID, mode: ${DETECTOR_ARG})${NC}"

# 7. Dashboard Gateway & Foxglove Bridge
echo -e "\n${CYAN}${BOLD}[7/7] Launching Operator Console & Telemetry Bridge...${NC}"

# Dashboard Gateway Node (Web Server + Safe Teleop Supervisor)
ros2 run cogproj_dashboard dashboard_node --ros-args \
    -p web_port:=8080 \
    -p target_cmd_vel_topic:=/cmd_vel_joy > "$LOG_DIR/dashboard.log" 2>&1 &
DASH_PID=$!
track_pid "$DASH_PID"

# Foxglove WebSocket Bridge (port 8765)
ros2 launch cogproj_bringup foxglove_bridge.launch.py \
    port:=8765 \
    address:=127.0.0.1 > "$LOG_DIR/foxglove_bridge.log" 2>&1 &
FOX_PID=$!
track_pid "$FOX_PID"

sleep 2
echo -e "${GREEN}[✓] Dashboard Gateway active on http://127.0.0.1:8080 (PID $DASH_PID)${NC}"
echo -e "${GREEN}[✓] Foxglove Bridge active on ws://127.0.0.1:8765 (PID $FOX_PID)${NC}"

# ------------------------------------------------------------------------------
# System Ready Banner & Operator Instructions
# ------------------------------------------------------------------------------
echo -e "\n${GREEN}${BOLD}======================================================================${NC}"
echo -e "${GREEN}${BOLD}      COGPROJ SYSTEM ONLINE & READY FOR LAPTOP CONNECTION            ${NC}"
echo -e "${GREEN}${BOLD}======================================================================${NC}"
echo -e "  • Control Mode      : ${BOLD}STOPPED (Motors Disarmed; Safe at Startup)${NC}"
echo -e "  • Detector Pipeline : ${BOLD}${DETECTOR_ARG^^} ONNX Inference${NC}"
echo -e "  • Web Dashboard URL : ${BOLD}http://localhost:8080${NC}"
echo -e "  • Foxglove URL      : ${BOLD}ws://localhost:8765${NC}"
echo -e "\n${CYAN}${BOLD}SSH Port Forwarding Procedure (Run on your laptop if not using VS Code Ports):${NC}"
echo -e "  ${BOLD}ssh -L 8080:127.0.0.1:8080 -L 8765:127.0.0.1:8765 veerobot@192.168.0.117${NC}"
echo -e "\n${YELLOW}${BOLD}Operator Instructions:${NC}"
echo -e "  1. Open http://localhost:8080 in your laptop browser."
echo -e "  2. The system starts in STOPPED mode. To drive manually, click '🎮 MANUAL / TELEOP'."
echo -e "  3. Use W/A/S/D keys to move (linear speed clamped to <= 0.10 m/s; angular <= 0.25 rad/s)."
echo -e "  4. Press SPACE or release keys to stop immediately."
echo -e "  5. Press '🚨 EMERGENCY STOP' to disarm motors and lock into STOPPED mode."
echo -e "  6. Press Ctrl+C in this terminal to shut down managed components cleanly."
echo -e "${GREEN}${BOLD}======================================================================${NC}\n"

# ------------------------------------------------------------------------------
# Process Supervisor Loop
# ------------------------------------------------------------------------------
while true; do
    # Verify child processes are alive
    if ! kill -0 "$DASH_PID" 2>/dev/null; then
        echo -e "${RED}[!] Dashboard Gateway exited unexpectedly. Check $LOG_DIR/dashboard.log${NC}"
        break
    fi
    if ! kill -0 "$PERC_PID" 2>/dev/null; then
        echo -e "${RED}[!] Perception pipeline exited unexpectedly. Check $LOG_DIR/perception.log${NC}"
        break
    fi
    sleep 2
done

cleanup
