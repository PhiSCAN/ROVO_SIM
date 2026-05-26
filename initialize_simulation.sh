#!/bin/bash
# initialize_simulation.sh [WORLD]
#
# WORLD: empty (default) | warehouse | pipeline | office | orchard |
#        solar_farm | construction
#
# Launches Zenoh + clearpath/empty world + RViz + rovo_fake_pico inside
# the navigation2:development container.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS_DIR="${SCRIPT_DIR}"

WORLD="${1:-${WORLD:-empty}}"

if [ ! -d "${SIM_WS_DIR}/src" ]; then
  echo "ERROR: ${SIM_WS_DIR}/src not found"
  exit 1
fi

if [ -d "${SIM_WS_DIR}/.git" ]; then
  git -C "${SIM_WS_DIR}" submodule update --init --recursive
fi

if [ -d "${SIM_WS_DIR}/src/realsense-ros" ]; then
  touch "${SIM_WS_DIR}/src/realsense-ros/COLCON_IGNORE"
fi

if [ "$(docker ps -aq -f name=^/initialize_simulation$)" ]; then
  docker rm -f initialize_simulation
fi

xhost +local: >/dev/null 2>&1 || true

MODEL="${MODEL:-tracked}"

echo "[initialize_simulation] world=${WORLD} model=${MODEL}"

docker run \
  --privileged \
  --net=host \
  --ipc=host \
  --name="initialize_simulation" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="${SIM_WS_DIR}:/opt/ROVO_SIM" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "$HOME/.Xauthority:/root/.Xauthority" \
  --device /dev/dri \
  navigation2:development \
  bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    cd /opt/ROVO_SIM && \
    colcon build --symlink-install && \
    source /opt/ROVO_SIM/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    export ROS_DOMAIN_ID=50 && \
    exec ros2 launch rovo_gz_description sim_bringup.launch.py \
      world:=${WORLD} model:=${MODEL}"
