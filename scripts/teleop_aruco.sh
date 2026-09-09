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

cd "${PROJECT_DIR}" && docker compose exec rovosim bash -c "\
  source /opt/ros/rolling/setup.bash && \
  source /opt/ros2_ws/install/setup.bash && \
  ros2 run joy joy_node __ns:=/aruco_bot __params:=/opt/ros2_ws/src/ROVO_SIM/rovo_fake_pico/config/joystick.yaml & \
  ros2 run rovo_fake_pico twist_stamped_relay __ns:=/aruco_bot __params:=/opt/ros2_ws/src/ROVO_SIM/rovo_fake_pico/config/twist_relay.yaml"
