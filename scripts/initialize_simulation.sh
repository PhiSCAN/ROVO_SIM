#!/bin/bash
# initialize_simulation.sh [WORLD] [MODEL]
#
# WORLD: empty (default) | warehouse | pipeline | office | orchard |
#        solar_farm | construction
# MODEL: tracked (default) | diff_drive
#
# Launches Zenoh + world + RViz + rovo_fake_pico via docker compose.
# Uses veera_simulator package with local assets (no clearpath_simulator dependency).
#
# NOTE: Run ./setup_network.sh once on the host to configure the loopback alias.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Stop running container if it exists, then start a fresh one
if [ -n "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Stopping existing rovosim container..."
  cd "${PROJECT_DIR}" && docker compose down
fi

echo "Starting rovosim container..."
"${SCRIPT_DIR}/run.sh"

WORLD="${1:-empty}"
MODEL="${2:-${MODEL:-tracked}}"

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
    exec ros2 launch veera_simulator veera_sim.launch.py \
    world:=${WORLD} model:=${MODEL} actors:=true"
