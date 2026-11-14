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
  cd "${PROJECT_DIR}" && docker compose down --timeout 0
fi

sim_launch_pid=""
cleanup() {
  status="${1:-$?}"
  trap - EXIT INT TERM
  echo "[initialize_simulation] stopping simulation stacks..."
  if [ -n "${sim_launch_pid}" ]; then
    kill -TERM "${sim_launch_pid}" 2>/dev/null || true
  fi
  (cd "${PROJECT_DIR}" && docker compose down --remove-orphans --timeout 0) || true
  exit "${status}"
}
on_exit() {
  status=$?
  cleanup "${status}"
}
trap on_exit EXIT
trap 'cleanup 130' INT
trap 'cleanup 143' TERM

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

cd "${PROJECT_DIR}" && USE_SIM_TIME=true docker compose exec -T rovosim bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    source /opt/sim/ros2_ws/install/setup.bash && \
    exec ros2 launch veera_simulator veera_sim.launch.py \
    world:=${WORLD} model:=${MODEL} actors:=false \
    use_sim_time:=${USE_SIM_TIME:-true}" &
sim_launch_pid=$!

echo "[initialize_simulation] waiting for simulator Zenoh router on port 7447..."
router_ready=0
for _ in $(seq 1 90); do
  if timeout 1 bash -c 'exec 3<>/dev/tcp/127.0.0.1/7447' 2>/dev/null; then
    router_ready=1
    break
  fi
  if ! kill -0 "${sim_launch_pid}" 2>/dev/null; then
    echo "ERROR: simulator launch exited before Zenoh became available." >&2
    wait "${sim_launch_pid}" || true
    exit 1
  fi
  sleep 1
done
if [ "${router_ready}" -ne 1 ]; then
  echo "ERROR: simulator Zenoh router did not become available on port 7447." >&2
  exit 1
fi

echo "[initialize_simulation] waiting for Gazebo simulated clock..."
clock_ready=0
for _ in $(seq 1 90); do
  clock_output="$(timeout 5 docker exec rovosim bash -lc \
      'source /opt/ros/rolling/setup.bash && source /opt/sim/ros2_ws/install/setup.bash && timeout -k 1 2 ros2 topic echo /clock --once' 2>&1 || true)"
  if printf '%s\n' "${clock_output}" | grep -Eq 'sec:|nanosec:'; then
    clock_ready=1
    break
  fi
  if ! kill -0 "${sim_launch_pid}" 2>/dev/null; then
    echo "ERROR: Gazebo launch exited before publishing /clock." >&2
    wait "${sim_launch_pid}" || true
    exit 1
  fi
  sleep 1
done
if [ "${clock_ready}" -ne 1 ]; then
  echo "ERROR: Gazebo did not publish simulated time on /clock." >&2
  exit 1
fi
echo "[initialize_simulation] Gazebo world is publishing simulated time."

wait "${sim_launch_pid}"
