#!/usr/bin/env bash
# =============================================================================
# setup_ws.sh — Unitree G1 Navigation Workspace Setup
#
# This script:
#   1. Sources ROS 2 Jazzy
#   2. Installs required apt packages (slam_toolbox, nav2, etc.)
#   3. Clones the official unitree_ros2 bridge into the workspace
#   4. Builds the workspace with colcon
#
# Usage:
#   chmod +x setup_ws.sh
#   ./setup_ws.sh
# =============================================================================
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"

echo "=== Unitree G1 Navigation Workspace Setup ==="
echo "Workspace: ${WORKSPACE_DIR}"
echo ""

# ---- 1. Source ROS 2 Jazzy --------------------------------------------------
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy not found at ${ROS_SETUP}."
    echo "Install ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"
echo "[OK] ROS 2 Jazzy sourced."

# ---- 2. Install APT Dependencies -------------------------------------------
echo ""
echo "=== Installing ROS 2 packages via apt ==="
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep
echo "[OK] APT packages installed."

# ---- 3. Initialize rosdep (if not already) ---------------------------------
if [[ ! -d "/etc/ros/rosdep/sources.list.d" ]]; then
    sudo rosdep init || true
fi
rosdep update --rosdistro=jazzy || true

# ---- 4. Clone unitree_ros2 -------------------------------------------------
echo ""
echo "=== Cloning unitree_ros2 ==="
mkdir -p "${SRC_DIR}"
if [[ -d "${SRC_DIR}/unitree_ros2" ]]; then
    echo "[SKIP] unitree_ros2 already exists at ${SRC_DIR}/unitree_ros2"
else
    git clone https://github.com/unitreerobotics/unitree_ros2.git "${SRC_DIR}/unitree_ros2"
    echo "[OK] unitree_ros2 cloned."
fi

# ---- 5. Install workspace rosdeps ------------------------------------------
echo ""
echo "=== Resolving rosdep dependencies ==="
cd "${WORKSPACE_DIR}"
rosdep install --from-paths src --ignore-src -r -y || true

# ---- 6. Set CycloneDDS as default RMW (required by Unitree SDK2) -----------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---- 7. Build the workspace ------------------------------------------------
echo ""
echo "=== Building workspace with colcon ==="
cd "${WORKSPACE_DIR}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo ""
echo "[OK] Workspace built successfully."

# ---- 8. Post-build instructions --------------------------------------------
echo ""
echo "============================================================"
echo "  Setup complete!"
echo ""
echo "  Add these lines to your ~/.bashrc:"
echo ""
echo "    source /opt/ros/jazzy/setup.bash"
echo "    source ${WORKSPACE_DIR}/install/setup.bash"
echo "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo ""
echo "  Then open a new terminal and run:"
echo ""
echo "    ros2 launch g1_autonomy_pkg g1_autonomy.launch.py"
echo "============================================================"
