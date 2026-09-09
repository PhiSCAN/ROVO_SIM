#!/bin/bash
# initialize_teleop.sh
# Exec into the running rovosim container and start
# teleop_twist_keyboard publishing TwistStamped on /cmd_vel_gcs.

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
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_gcs \
    -p stamped:=true \
    -p repeat_rate:=30.0 \
    -p key_timeout:=0.6 \
    -p speed:=2.0 \
    -p turn:=1.0"
