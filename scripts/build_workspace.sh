#!/usr/bin/env bash
# ==============================================================================
# CogProj Workspace Build Script
# Builds CogProj packages in dependency order, sources overlay, and tests
# ==============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

RUN_TESTS="false"
PACKAGE_TARGET=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --test|-t)
            RUN_TESTS="true"
            shift
            ;;
        --package|-p)
            PACKAGE_TARGET="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: ./scripts/build_workspace.sh [--test] [--package <name>]"
            exit 1
            ;;
    esac
done

echo "======================================================================"
echo "          COGPROJ REPRODUCIBLE WORKSPACE BUILDER"
echo "======================================================================"

cd "$WS_DIR"

# 1. Environment sourcing
echo "[1/4] Sourcing ROS 2 Jazzy and external robot underlays..."
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "/home/veerobot/lyra_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/home/veerobot/lyra_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=155
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. Build Execution
echo "[2/4] Building workspace packages via colcon..."
if [[ -n "$PACKAGE_TARGET" ]]; then
    echo "[i] Building targeted package: $PACKAGE_TARGET"
    colcon build --symlink-install --packages-select "$PACKAGE_TARGET"
else
    # Build interfaces first to guarantee code generation before dependent nodes
    echo "[i] Stage 1: Building interfaces..."
    colcon build --symlink-install --packages-select cogproj_interfaces

    echo "[i] Stage 2: Sourcing interfaces and building all packages..."
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
    colcon build --symlink-install --packages-up-to \
        cogproj_detector_base \
        cogproj_detection \
        cogproj_camera \
        cogproj_tracking \
        cogproj_counting \
        cogproj_spatial \
        cogproj_visualization \
        cogproj_bringup \
        cogproj_dashboard
fi

echo "[✓] Build completed successfully."

# 3. Source Resulting Overlay
echo "[3/4] Sourcing newly built overlay..."
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
echo "[✓] CogProj overlay sourced."

# 4. Verification Tests
if [[ "$RUN_TESTS" == "true" ]]; then
    echo "[4/4] Executing automated regression tests..."
    if [[ -n "$PACKAGE_TARGET" ]]; then
        colcon test --packages-select "$PACKAGE_TARGET"
    else
        colcon test --packages-select \
            cogproj_camera \
            cogproj_detector_base \
            cogproj_detection \
            cogproj_tracking \
            cogproj_counting \
            cogproj_spatial \
            cogproj_visualization \
            cogproj_bringup \
            cogproj_dashboard
    fi
    colcon test-result --all --verbose
    echo "[✓] All tests verified green."
else
    echo "[4/4] Skipping tests (pass --test to run automated tests)."
fi

echo "======================================================================"
echo "[✓] Workspace is ready and up to date."
echo "======================================================================"

