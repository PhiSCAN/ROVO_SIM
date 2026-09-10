#!/bin/bash
# initialize_simulation.sh [WORLD]
#
# WORLD: empty (default) | warehouse | pipeline | office | orchard |
#        solar_farm | construction
#
# Launches Zenoh + clearpath world + RViz + rovo_fake_pico via docker compose.
#
# NOTE: Run ./setup_network.sh once on the host to configure the loopback alias.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Ensure container is running
if [ -z "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Starting rovosim container..."
  "${SCRIPT_DIR}/run.sh"
fi

WORLD="${1:-empty}"
MODEL="${MODEL:-tracked}"

echo "[initialize_simulation] world=${WORLD} model=${MODEL}"

xhost +local: >/dev/null 2>&1 || true

# Check if workspace has been built in the container's mounted volume
cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "
  if [ ! -d /opt/sim/ros2_ws/build ] || [ ! -d /opt/sim/ros2_ws/install ] || [ ! -d /opt/sim/ros2_ws/log ]; then
    echo 'Workspace not built. Running colcon build...'
    exit 1
  fi
" 2>/dev/null

if [ $? -ne 0 ]; then
  echo "Building workspace..."
  "${SCRIPT_DIR}/colcon_build.sh"
fi

cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    source /opt/sim/ros2_ws/install/setup.bash && \
    exec ros2 launch rovo_gz_description sim_bringup.launch.py \
    world:=${WORLD} model:=${MODEL}"
