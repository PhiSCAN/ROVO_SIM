#!/bin/bash
# initialize_simulation.sh [WORLD] [MODEL]
#
# WORLD: name of a world in ros2_ws/src/veera_simulator/worlds (default: empty)
#        e.g. empty | warehouse | pipeline | office | orchard | solar_farm | construction
# MODEL: tracked (default) | diff_drive
#
# Usage: ./initialize_simulation.sh [WORLD] [MODEL]
#
# Launches Zenoh + world + RViz + rovo_fake_pico via docker compose.
# Uses veera_simulator package with local assets (no clearpath_simulator dependency).
#
# NOTE: Run ./setup_network.sh once on the host to configure the loopback alias.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

WORLD="${1:-empty}"
MODEL="${2:-${MODEL:-tracked}}"

# Validate world against veera_simulator/worlds before starting the container
WORLDS_DIR="${PROJECT_DIR}/ros2_ws/src/veera_simulator/worlds"
if [ ! -f "${WORLDS_DIR}/${WORLD}.sdf" ]; then
  AVAILABLE="$(find "${WORLDS_DIR}" -maxdepth 1 -name '*.sdf' ! -name 'walking_actor*' \
    -printf '%f ' | sed 's/\.sdf / /g')"
  echo "ERROR: world '${WORLD}' not found in ${WORLDS_DIR}" >&2
  echo "Available worlds: ${AVAILABLE}" >&2
  exit 1
fi

# Stop running container if it exists, then start a fresh one
if [ -n "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Stopping existing rovosim container..."
  cd "${PROJECT_DIR}" && docker compose down
fi

echo "Starting rovosim container..."
"${SCRIPT_DIR}/run.sh"

echo "[initialize_simulation] world=${WORLD} model=${MODEL}"

xhost +local: >/dev/null 2>&1 || true

# Check if workspace has been built in the container's mounted volume
# (if ! ... guard keeps `set -e` from exiting before the rebuild branch)
if ! cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "
  if [ ! -d /opt/sim/ros2_ws/build ] || [ ! -d /opt/sim/ros2_ws/install ] || [ ! -d /opt/sim/ros2_ws/log ]; then
    echo 'Workspace not built. Running colcon build...'
    exit 1
  fi
" 2>/dev/null; then
  echo "Building workspace..."
  "${SCRIPT_DIR}/colcon_build.sh"
fi

cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    source /opt/sim/ros2_ws/install/setup.bash && \
    exec ros2 launch veera_simulator veera_sim.launch.py \
    world:=${WORLD} model:=${MODEL} actors:=true"
