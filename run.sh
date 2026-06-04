#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker rm -f rovo-sim 2>/dev/null || true
xhost +local: >/dev/null 2>&1 || true

# Gazebo Fuel model cache vendored in-repo (gz_cache/), mounted to /root/.gz.
mkdir -p "${SCRIPT_DIR}/gz_cache"

docker run -it \
  --privileged \
  --runtime=nvidia \
  --net=host \
  --ipc=host \
  --name="rovo-sim" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="${SCRIPT_DIR}/src:/opt/simulation_ws/src" \
  --volume="${SCRIPT_DIR}/gz_cache:/root/.gz" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "$HOME/.Xauthority:/root/.Xauthority" \
  --device /dev/dri \
  navigation2:development \
  bash
