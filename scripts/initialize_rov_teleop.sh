#!/bin/bash
# initialize_teleop.sh
# Exec into the running initialize_simulation container and start
# teleop_twist_keyboard publishing TwistStamped on /cmd_vel_gcs.

set -e

exec docker compose exec -it rovosim bash -c "\
  source /opt/ros/rolling/setup.bash && \
  source /opt/sim/ros2_ws/install/setup.bash && \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_gcs \
    -p stamped:=true \
    -p repeat_rate:=30.0 \
    -p key_timeout:=0.6 \
    -p speed:=2.0 \
    -p turn:=1.0"
