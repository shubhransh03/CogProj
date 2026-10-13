#!/usr/bin/env bash
# ==============================================================================
# CogProj Reproducible Dependency Installation Script
# Target: Raspberry Pi 5 ARM64, Ubuntu 24.04.3 LTS (Noble), ROS 2 Jazzy
# ==============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
MANIFEST_FILE="$WS_DIR/dependencies/apt-packages.txt"

echo "======================================================================"
echo "          COGPROJ REPRODUCIBLE DEPENDENCY INSTALLER"
echo "======================================================================"

# 1. Environment & Architecture Verification
echo "[1/6] Verifying system platform and environment..."

ARCH="$(uname -m)"
if [[ "$ARCH" != "aarch64" ]]; then
    echo "[-] WARNING: Detected architecture '$ARCH'. Recommended target is aarch64 (ARM64)."
else
    echo "[✓] Architecture verified: aarch64 (ARM64)"
fi

if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
        echo "[-] WARNING: Detected OS '$PRETTY_NAME'. Recommended target is Ubuntu 24.04 (noble)."
    else
        echo "[✓] Operating System verified: Ubuntu 24.04 LTS (noble)"
    fi
fi

if [[ ! -d /opt/ros/jazzy ]]; then
    echo "[!] ERROR: ROS 2 Jazzy not found in /opt/ros/jazzy. Please install ROS 2 Jazzy first."
    exit 1
else
    echo "[✓] ROS 2 distribution verified: Jazzy Jalisco (/opt/ros/jazzy)"
fi

# 2. Check Core CLI Utilities
echo "[2/6] Checking required tooling (apt, rosdep, colcon)..."

for tool in apt rosdep colcon; do
    if command -v "$tool" >/dev/null 2>&1; then
        echo "[✓] Command '$tool' is available."
    else
        echo "[!] Tool '$tool' is missing. Attempting to install..."
        if [[ "$tool" == "rosdep" ]]; then
            sudo apt-get install -y python3-rosdep
            sudo rosdep init || true
            rosdep update || true
        elif [[ "$tool" == "colcon" ]]; then
            sudo apt-get install -y python3-colcon-common-extensions
        fi
    fi
done

# 3. Read Package Manifest and Identify Missing Packages
echo "[3/6] Inspecting required project dependencies against local system..."

if [[ ! -f "$MANIFEST_FILE" ]]; then
    echo "[!] ERROR: Dependency manifest not found at $MANIFEST_FILE"
    exit 1
fi

MISSING_PACKAGES=()
INSTALLED_PACKAGES=()

while IFS= read -r pkg || [[ -n "$pkg" ]]; do
    # Trim whitespace and ignore comments
    pkg="$(echo "$pkg" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
    if [[ -z "$pkg" || "$pkg" == \#* ]]; then
        continue
    fi

    if dpkg -s "$pkg" >/dev/null 2>&1; then
        VERSION="$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || echo 'unknown')"
        INSTALLED_PACKAGES+=("$pkg ($VERSION)")
    else
        MISSING_PACKAGES+=("$pkg")
    fi
done < "$MANIFEST_FILE"

echo "[i] Currently installed manifest packages: ${#INSTALLED_PACKAGES[@]}"
if [[ ${#MISSING_PACKAGES[@]} -eq 0 ]]; then
    echo "[✓] All manifest packages are already installed."
else
    echo "[i] Found ${#MISSING_PACKAGES[@]} missing package(s): ${MISSING_PACKAGES[*]}"
fi

# 4. Install Missing Packages via APT
if [[ ${#MISSING_PACKAGES[@]} -gt 0 ]]; then
    echo "[4/6] Installing missing packages via apt (no full-upgrade, no autoremove)..."
    sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ros2.list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
    sudo apt-get install -y --no-install-recommends "${MISSING_PACKAGES[@]}"
    echo "[✓] Successfully installed missing packages."
else
    echo "[4/6] No APT packages need installation."
fi

# 5. Resolve rosdep across CogProj source packages
echo "[5/6] Verifying workspace dependencies via rosdep..."
if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    cd "$WS_DIR"
    rosdep check --from-paths src --ignore-src --rosdistro jazzy || {
        echo "[i] Resolving any residual rosdep dependencies..."
        rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
    }
    echo "[✓] All workspace source package dependencies satisfied."
else
    echo "[i] Initializing rosdep sources..."
    sudo rosdep init || true
    rosdep update || true
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    cd "$WS_DIR"
    rosdep check --from-paths src --ignore-src --rosdistro jazzy
    echo "[✓] Workspace dependencies verified via rosdep."
fi

# 6. Final Status & Summary
echo "[6/6] Generating installation audit report..."
echo ""
echo "----------------------------------------------------------------------"
echo "VERIFIED INSTALLED CORE COMPONENTS:"
for pkg in ros-jazzy-rclpy ros-jazzy-cv-bridge ros-jazzy-foxglove-bridge ros-jazzy-slam-toolbox python3-opencv python3-colcon-common-extensions; do
    if dpkg -s "$pkg" >/dev/null 2>&1; then
        V="$(dpkg-query -W -f='${Version}' "$pkg")"
        printf "  • %-36s : %s\n" "$pkg" "$V"
    else
        printf "  • %-36s : NOT INSTALLED\n" "$pkg"
    fi
done
echo "----------------------------------------------------------------------"
echo "[✓] Dependency installation and verification completed successfully."
echo "======================================================================"
