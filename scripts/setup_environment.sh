#!/usr/bin/env bash
# ==============================================================================
# CogProj Environment Configuration Script
# Sets up ROS 2 Jazzy, Domain ID 155, FastRTPS, and Workspace Overlays
# Usage: source scripts/setup_environment.sh
# ==============================================================================

# Ensure script is being sourced, not executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[!] Warning: This script must be sourced, not executed."
    echo "    Run: source ${BASH_SOURCE[0]}"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# 1. Base ROS 2 Installation
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    echo "[!] ERROR: /opt/ros/jazzy/setup.bash not found."
    return 1
fi

# 2. DDS Network Configuration
export ROS_DOMAIN_ID=155
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. Robot Platform Underlay (Lyra / Beetle Bot Hardware Drivers)
LYRA_SETUP="/home/veerobot/lyra_ws/install/setup.bash"
if [[ -f "$LYRA_SETUP" ]]; then
    # shellcheck disable=SC1091
    source "$LYRA_SETUP"
fi

# 4. CogProj Workspace Overlay
COGPROJ_SETUP="$WS_DIR/install/setup.bash"
if [[ -f "$COGPROJ_SETUP" ]]; then
    # shellcheck disable=SC1091
    source "$COGPROJ_SETUP"
fi

# 5. Diagnostic Echo
echo "[✓] CogProj Environment Configured:"
echo "    • ROS_DISTRO         : ${ROS_DISTRO:-jazzy}"
echo "    • ROS_DOMAIN_ID      : $ROS_DOMAIN_ID"
echo "    • RMW_IMPLEMENTATION : $RMW_IMPLEMENTATION"
echo "    • Underlay           : ${LYRA_SETUP} ($([[ -f "$LYRA_SETUP" ]] && echo 'active' || echo 'not found'))"
echo "    • CogProj Overlay    : ${COGPROJ_SETUP} ($([[ -f "$COGPROJ_SETUP" ]] && echo 'active' || echo 'pending build'))"

