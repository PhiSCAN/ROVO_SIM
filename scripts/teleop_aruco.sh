#!/bin/bash
# teleop_aruco.sh
# Launch teleop for the aruco marker bot via joy node and twist node

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# # First run the simulation initialization script in the background
# "${SCRIPT_DIR}/initialize_simulation.sh" &

cd "${PROJECT_DIR}" && USE_SIM_TIME=true docker compose exec rovosim bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    source /opt/sim/ros2_ws/install/setup.bash && \
  ros2 launch rovo_gz_description spawn_aruco.launch.py \
    use_sim_time:=${USE_SIM_TIME:-true}"
