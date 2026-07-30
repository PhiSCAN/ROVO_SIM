#!/bin/bash
# initialize_simulation.sh [WORLD] [headless]
#
# WORLD: empty (default) | autoware | shinjuku | warehouse | pipeline |
#        office | orchard | solar_farm | construction
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

docker rm -f initialize_simulation 2>/dev/null || true

# Bind robot IP 192.168.1.200 to host loopback so the in-container Zenoh
# router (listening on tcp/[::]:7447) is reachable at tcp/192.168.1.200:7447,
# matching ZENOH_CONFIG_OVERRIDE in the navigation2:development image.
if ! ip addr show lo | grep -q "192.168.1.200"; then
  sudo ip addr add 192.168.1.200/32 dev lo 2>/dev/null \
    || echo "WARN: could not add 192.168.1.200 loopback alias (need sudo)"
fi

xhost +local: >/dev/null 2>&1 || true

# Gazebo Fuel model cache vendored in-repo (gz_cache/) and mounted to
# /root/.gz so warehouse's 27 Fuel models load locally, no per-launch fetch.
mkdir -p "${SIM_WS_DIR}/gz_cache"

MODEL="${MODEL:-tracked}"

# RealSense RGBD camera OFF by default (its 640x480@30Hz GPU->CPU readback is
# the main gz-server RTF cost). Enable: REALSENSE=true ./initialize_simulation.sh shinjuku
REALSENSE="${REALSENSE:-false}"

# Headless: gz server-only, no GUI window. Also accepted as 2nd positional arg
# (`./initialize_simulation.sh shinjuku headless`) or HEADLESS=true env.
HEADLESS="${HEADLESS:-false}"
case "${2:-}" in
  headless|--headless|-s) HEADLESS=true ;;
esac

echo "[initialize_simulation] world=${WORLD} model=${MODEL} realsense=${REALSENSE} headless=${HEADLESS}"

docker run \
  --privileged \
  --gpus all \
  --net=host \
  --ipc=host \
  --name="initialize_simulation" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=__NV_PRIME_RENDER_OFFLOAD=1 \
  --env=__GLX_VENDOR_LIBRARY_NAME=nvidia \
  --env=__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  --volume="${SIM_WS_DIR}:/opt/ROVO_SIM" \
  --volume="${SIM_WS_DIR}/gz_cache:/root/.gz" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "$HOME/.Xauthority:/root/.Xauthority" \
  --device /dev/dri \
  navigation2:development \
  bash -c "set -e && \
    source /opt/ros/rolling/setup.bash && \
    cd /opt/ROVO_SIM && \
    rm -f /root/.gz/sim/*/gui.config && \
    colcon build --symlink-install && \
    source /opt/ROVO_SIM/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
    export ROS_DOMAIN_ID=50 && \
    exec ros2 launch rovo_gz_description sim_bringup.launch.py \
      world:=${WORLD} model:=${MODEL} realsense:=${REALSENSE} \
      headless:=${HEADLESS}"
