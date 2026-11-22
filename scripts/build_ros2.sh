#!/usr/bin/env bash
# Build the ROS 2 workspace (ament_python package unitree_g1_social_hri).
set -euo pipefail
. "$(dirname "$0")/common.sh"
source_ros
cd "$WS"
colcon build --symlink-install --packages-select unitree_g1_social_hri
echo
echo "Built. In every new terminal run:"
echo "  source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
