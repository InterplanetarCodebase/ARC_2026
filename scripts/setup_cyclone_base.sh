#!/bin/bash
# setup_cyclone_base.sh — Source this on the base station before running
# Usage: source scripts/setup_cyclone_base.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$REPO_ROOT/ros2_ws/src/interplanetar_rover/config/cyclonedds.xml"
export ROS_DOMAIN_ID=42
echo "[BASE] ROS2 Humble + CycloneDDS ready  |  DOMAIN_ID=$ROS_DOMAIN_ID"
