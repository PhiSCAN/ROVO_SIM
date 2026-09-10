#!/bin/bash
# colcon_build.sh
#
# Runs rosdep update + rosdep install + colcon build inside the
# rovosim container against the in-repo ros2_ws workspace.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Ensure container is running
if [ -z "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Starting rovosim container..."
  "${SCRIPT_DIR}/run.sh"
fi

if [ ! -d "${PROJECT_DIR}/ros2_ws/src" ]; then
  echo "ERROR: ${PROJECT_DIR}/ros2_ws/src not found"
  exit 1
fi

if [ -d "${PROJECT_DIR}/.git" ]; then
  git -C "${PROJECT_DIR}" submodule update --init --recursive
fi

# Clean build artifacts inside the container
# Note: ros2_ws is mounted from host, so build/install/log will persist on host
cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "set -e && \
  source /opt/ros/rolling/setup.bash && \
  cd /opt/sim/ros2_ws && \
  rm -rf build install log && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --symlink-install"
