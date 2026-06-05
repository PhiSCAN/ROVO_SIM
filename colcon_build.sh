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

# Clean runs inside the container as root: build/install/log are created
# root-owned, so a host-side rm would hit permission denied.
docker run \
  --name="colcon_build" \
  --volume="${SIM_WS_DIR}:/opt/ROVO_SIM" \
  navigation2:development \
  bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    cd /opt/ROVO_SIM && \
    rm -rf build install log && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"
