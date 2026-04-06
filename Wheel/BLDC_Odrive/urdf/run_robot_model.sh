#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
URDF_PATH="${SCRIPT_DIR}/urdf/rover_visual.urdf"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS2 environment first."
  exit 1
fi

if [[ ! -f "${URDF_PATH}" ]]; then
  echo "URDF not found: ${URDF_PATH}"
  exit 1
fi

echo "Starting robot_state_publisher with ${URDF_PATH}"
ros2 run robot_state_publisher robot_state_publisher "${URDF_PATH}"
