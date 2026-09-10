#!/bin/bash
# teleop_aruco.sh
# Launch teleop for the aruco marker bot via joy node and twist node

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Ensure container is running
if [ -z "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Starting rovosim container..."
  "${SCRIPT_DIR}/run.sh"
fi

if [ -z "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "ERROR: service 'rovosim' not running."
  exit 1
fi

cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    source /opt/sim/ros2_ws/install/setup.bash && \
  ros2 launch rovo_gz_description spawn_aruco.launch.py"
