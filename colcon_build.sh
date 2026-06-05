#!/bin/bash
# colcon_build.sh
#
# Runs rosdep update + rosdep install + colcon build inside the
# navigation2:development container against the in-repo src/ workspace.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS_DIR="${SCRIPT_DIR}"

if [ ! -d "${SIM_WS_DIR}/src" ]; then
  echo "ERROR: ${SIM_WS_DIR}/src not found"
  exit 1
fi

if [ -d "${SIM_WS_DIR}/.git" ]; then
  git -C "${SIM_WS_DIR}" submodule update --init --recursive
fi

docker rm -f colcon_build 2>/dev/null || true

docker run \
  --name="colcon_build" \
  --volume="${SIM_WS_DIR}:/opt/simulation_ws" \
  navigation2:development \
  bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    cd /opt/simulation_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"
