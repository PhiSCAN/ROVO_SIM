#!/bin/bash
# initialize_teleop.sh
# Exec into the running initialize_simulation container and start
# teleop_twist_keyboard publishing TwistStamped on /cmd_vel_gcs.

set -e

if [ -z "$(docker ps -q -f name=^/initialize_simulation$)" ]; then
  echo "ERROR: container 'initialize_simulation' not running."
  echo "Start it first: ./initialize_simulation.sh"
  exit 1
fi

exec docker exec -it initialize_simulation bash -c "\
  source /opt/ros/rolling/setup.bash && \
  source /opt/ROVO_SIM/install/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
  export ROS_DOMAIN_ID=50 && \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_gcs \
    -p stamped:=true \
    -p repeat_rate:=30.0 \
    -p key_timeout:=0.6"
