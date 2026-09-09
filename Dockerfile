# ROS 2 Rolling + Gazebo Sim Dockerfile for Rovo Simulation
# 
# All ROS packages are in the host's ros2_ws/src folder and mounted into the container.
# No packages are built in the Dockerfile - they are built on the host or in the container
# via the mounted workspace.
#
# Build:    docker build -t rovosim:latest .
# Run:      docker run -it --rm --gpus all -p 8080:8080 rovosim:latest
#
# For better GPU performance:
#   docker run -it --rm --gpus all --ipc=host --ulimit memlock=-1 --ulimit stack=-1 rovosim:latest

# Configuration - change these to customize the build
ARG ROS2_DISTRO=rolling
ARG GZ_VERSION=rolling

# =============================================================================
# Single stage: Base image with ROS 2 and Gazebo Sim + development tools
# =============================================================================
FROM osrf/ros:${ROS2_DISTRO}-desktop

# Set environment variables
ENV ROS_DOMAIN_ID=0 \
    DEBIAN_FRONTEND=noninteractive

# Create workspace structure
RUN mkdir -p /opt/ros2_ws/src /opt/ros2_ws/build /opt/ros2_ws/install /opt/ros2_ws/log
WORKDIR /opt/ros2_ws

# Install additional dependencies needed for development
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    vim \
    nano \
    tmux \
    htop \
    net-tools \
    iproute2 \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    ros-rolling-rmw-zenoh-cpp \
    ros-rolling-gz-ros2-control \
    ros-rolling-ros-gz \
    ros-rolling-xacro \
    ros-rolling-joint-state-publisher-gui \
    ros-rolling-ros-gz-sim \
    ros-rolling-ros-gz-bridge \
    ros-rolling-gz-utils-vendor \
    ros-rolling-gz-math-vendor \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --break-system-packages numpy orjson

# Setup automatic environment sourcing in root user's .bashrc
# Use single quotes or no quotes for ARG variable substitution to work
RUN echo source /opt/ros/rolling/setup.bash >> /root/.bashrc && \
    echo source /opt/ros2_ws/install/setup.bash >> /root/.bashrc

WORKDIR /opt/ros2_ws
CMD ["bash"]
