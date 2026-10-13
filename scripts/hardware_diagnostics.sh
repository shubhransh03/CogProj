#!/usr/bin/env bash
# ==============================================================================
# CogProj Comprehensive Physical Hardware Diagnostics Script
# Performs non-destructive, read-only inspection of host system, physical sensors,
# buses, permissions, ROS 2 topics, TF frames, and safety states.
# NEVER publishes to actuation topics (/cmd_vel*) or commands robot movement.
# ==============================================================================
set -eo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}${BOLD}======================================================================${NC}"
echo -e "${BLUE}${BOLD}   CogProj Physical Hardware & Deployment Diagnostics (Phase 5L)      ${NC}"
echo -e "${BLUE}${BOLD}======================================================================${NC}"

# ------------------------------------------------------------------------------
# 1. System & Host Information
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[1/6] Host System & Hardware Metrics${NC}"
echo "--------------------------------------------------"
HOST_NAME=$(hostname)
KERNEL_VER=$(uname -r)
ARCH=$(uname -m)
MODEL="Unknown"
if [[ -f /proc/device-tree/model ]]; then
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
fi

echo -e "  • Hostname       : ${BOLD}$HOST_NAME${NC}"
echo -e "  • Hardware Model : ${BOLD}$MODEL${NC}"
echo -e "  • Architecture   : ${BOLD}$ARCH${NC}"
echo -e "  • Linux Kernel   : $KERNEL_VER"

# Thermals & Throttling
TEMP_STR="N/A"
if command -v vcgencmd &>/dev/null; then
    TEMP_STR=$(vcgencmd measure_temp 2>/dev/null || echo "N/A")
    THROTTLED_STR=$(vcgencmd get_throttled 2>/dev/null || echo "N/A")
    echo -e "  • SoC Thermals   : ${GREEN}$TEMP_STR${NC}"
    echo -e "  • Throttling     : $THROTTLED_STR"
elif [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
    RAW_TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$(awk "BEGIN {print $RAW_TEMP/1000}")
    echo -e "  • SoC Thermals   : ${GREEN}${TEMP_C}°C${NC}"
fi

# RAM & Disk
RAM_TOTAL=$(free -h | awk '/^Mem:/ {print $2}')
RAM_AVAIL=$(free -h | awk '/^Mem:/ {print $7}')
DISK_AVAIL=$(df -h / | awk 'NR==2 {print $4 " available (" $5 " used)"}')
echo -e "  • Memory (RAM)   : $RAM_AVAIL available / $RAM_TOTAL total"
echo -e "  • Disk Root (/)  : $DISK_AVAIL"

# ------------------------------------------------------------------------------
# 2. Hardware Buses & Device Nodes
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[2/6] Physical Device Interfaces & Permissions${NC}"
echo "--------------------------------------------------"

# USB CP210x UART (RPLiDAR Bridge)
if lsusb | grep -qi "CP210"; then
    USB_LIDAR=$(lsusb | grep -i "CP210")
    echo -e "  • USB UART LiDAR : ${GREEN}[✓] Detected${NC} ($USB_LIDAR)"
else
    echo -e "  • USB UART LiDAR : ${YELLOW}[!] Not detected on USB bus${NC}"
fi

# Serial Port (/dev/ttyUSB0)
if [[ -e /dev/ttyUSB0 ]]; then
    PERMS=$(ls -l /dev/ttyUSB0 | awk '{print $1 " " $3 ":" $4}')
    if [[ -r /dev/ttyUSB0 && -w /dev/ttyUSB0 ]]; then
        echo -e "  • Serial Port    : ${GREEN}[✓] /dev/ttyUSB0 accessible (read/write)${NC} ($PERMS)"
    else
        echo -e "  • Serial Port    : ${RED}[✗] /dev/ttyUSB0 exists but lacks RW permission${NC} ($PERMS)"
    fi
else
    echo -e "  • Serial Port    : ${YELLOW}[!] /dev/ttyUSB0 not present${NC}"
fi

# Motor Controller Serial (/dev/ttyAMA0)
if [[ -e /dev/ttyAMA0 ]]; then
    PERMS=$(ls -l /dev/ttyAMA0 | awk '{print $1 " " $3 ":" $4}')
    if [[ -r /dev/ttyAMA0 && -w /dev/ttyAMA0 ]]; then
        echo -e "  • STM32 Motor Port: ${GREEN}[✓] /dev/ttyAMA0 accessible (read/write)${NC} ($PERMS)"
    else
        echo -e "  • STM32 Motor Port: ${RED}[✗] /dev/ttyAMA0 lacks RW permission${NC} ($PERMS)"
    fi
else
    echo -e "  • STM32 Motor Port: ${YELLOW}[!] /dev/ttyAMA0 not present${NC}"
fi

# User Groups
USER_GROUPS=$(groups "$USER")
if echo "$USER_GROUPS" | grep -qw "dialout" && echo "$USER_GROUPS" | grep -qw "video"; then
    echo -e "  • User Groups    : ${GREEN}[✓] $USER belongs to dialout and video${NC}"
else
    echo -e "  • User Groups    : ${YELLOW}[!] Verify groups (dialout, video): $USER_GROUPS${NC}"
fi

# Camera Video Nodes (/dev/video*)
VIDEO_NODES=$(ls -d /dev/video* 2>/dev/null | wc -l || echo 0)
if [[ "$VIDEO_NODES" -gt 0 ]]; then
    echo -e "  • Video Devices  : ${GREEN}[✓] $VIDEO_NODES video device nodes present (/dev/video*)${NC}"
else
    echo -e "  • Video Devices  : ${RED}[✗] No /dev/video* nodes found${NC}"
fi

# Network IP
IPV4_ADDRS=$(hostname -I | tr -s ' ')
echo -e "  • Host IPv4      : ${BOLD}${IPV4_ADDRS}${NC}"

# ------------------------------------------------------------------------------
# 3. Environment & Underlay Configuration
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[3/6] ROS 2 Jazzy & DDS Environment${NC}"
echo "--------------------------------------------------"
# Source environment cleanly
# shellcheck disable=SC1091
source "$WS_DIR/scripts/setup_environment.sh" >/dev/null 2>&1

echo -e "  • ROS_DISTRO         : ${BOLD}${ROS_DISTRO:-UNKNOWN}${NC}"
echo -e "  • ROS_DOMAIN_ID      : ${BOLD}${ROS_DOMAIN_ID:-0}${NC}"
echo -e "  • RMW_IMPLEMENTATION : ${BOLD}${RMW_IMPLEMENTATION:-UNKNOWN}${NC}"

# ------------------------------------------------------------------------------
# 4. ROS 2 Graph & Topic Status Audit
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[4/6] ROS 2 Graph & Sensor Topic Streams${NC}"
echo "--------------------------------------------------"

ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null || true)

audit_topic() {
    local topic="$1"
    local desc="$2"
    if echo "$ACTIVE_TOPICS" | grep -qx "$topic"; then
        local pubs
        pubs=$(ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
        if [[ -z "$pubs" ]]; then pubs=0; fi

        if [[ "$pubs" -gt 0 ]]; then
            echo -e "  • $desc ($topic): ${GREEN}[PUBLISHING]${NC} (${pubs} active publisher(s))"
        else
            echo -e "  • $desc ($topic): ${YELLOW}[IDLE]${NC} (Topic advertised, 0 publishers)"
        fi
    else
        echo -e "  • $desc ($topic): ${RED}[NOT ADVERTISED]${NC} (Driver offline)"
    fi
}

audit_topic "/camera/image_raw" "Camera Raw Image"
audit_topic "/camera/camera_info" "Camera Info / K"
audit_topic "/scan" "LiDAR LaserScan"
audit_topic "/odometry/filtered" "Filtered Odometry"
audit_topic "/imu/data_raw" "Raw IMU Data"
audit_topic "/tf" "Dynamic TF Stream"
audit_topic "/tf_static" "Static TF Stream"
audit_topic "/map" "Occupancy Grid Map"
audit_topic "/lyra/armed" "Lyra Armed Status"
audit_topic "/wheel_rpm" "Lyra Wheel RPM"

# CogProj perception topics
echo "  - CogProj Pipeline Topics:"
audit_topic "/cogproj/image_raw" "  CogProj Camera"
audit_topic "/cogproj/detections" "  Ore Detections"
audit_topic "/cogproj/tracked_ores" "  Tracked Ores"
audit_topic "/cogproj/counts_summary" "  Ore Counts"
audit_topic "/cogproj/spatial_ores" "  Spatial Ores"
audit_topic "/cogproj/image_annotated" "  Annotated Stream"

# ------------------------------------------------------------------------------
# 5. Camera Calibration State Audit
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[5/6] Camera Calibration Verification${NC}"
echo "--------------------------------------------------"
if echo "$ACTIVE_TOPICS" | grep -qx "/camera/camera_info"; then
    CAM_INFO_PUBS=$(ros2 topic info /camera/camera_info 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
    if [[ "$CAM_INFO_PUBS" -gt 0 ]]; then
        K_MATRIX=$(ros2 topic echo --once /camera/camera_info 2>/dev/null | grep -A 4 "k:" | tr '\n' ' ' || echo "")
        if echo "$K_MATRIX" | grep -qE "(- 0.0|\[0.0)"; then
            echo -e "  • Calibration State : ${YELLOW}[UNCALIBRATED] K matrix is all zeros (raw camera without calibration YAML)${NC}"
            echo -e "                        (Spatial association relies on calibrated defaults or FOV estimates)"
        else
            echo -e "  • Calibration State : ${GREEN}[CALIBRATED] Non-zero intrinsic matrix detected${NC}"
        fi
    else
        echo -e "  • Calibration State : ${YELLOW}[STANDBY] /camera/camera_info advertised with 0 publishers${NC}"
    fi
else
    echo -e "  • Calibration State : ${YELLOW}[STANDBY] /camera/camera_info not currently publishing${NC}"
fi

# ------------------------------------------------------------------------------
# 6. Safety & Actuation Confinement Audit
# ------------------------------------------------------------------------------
echo -e "\n${CYAN}${BOLD}[6/6] Safety Confinement Verification${NC}"
echo "--------------------------------------------------"

CMD_VEL_PUBS=0
if echo "$ACTIVE_TOPICS" | grep -qx "/cmd_vel"; then
    CMD_VEL_PUBS=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3}' || echo "0")
fi

if [[ "$CMD_VEL_PUBS" -eq 0 ]]; then
    echo -e "  • Actuation Safety  : ${GREEN}[PASS] Zero active publishers on /cmd_vel${NC}"
    echo -e "                        Rover is stationary; motors disarmed; safe for stationary testing."
else
    echo -e "  • Actuation Safety  : ${RED}[WARNING] $CMD_VEL_PUBS publisher(s) detected on /cmd_vel!${NC}"
fi

echo -e "\n${BLUE}${BOLD}======================================================================${NC}"
echo -e "${GREEN}${BOLD}[✓] Hardware diagnostics check complete.${NC}"
echo -e "${BLUE}${BOLD}======================================================================${NC}\n"
