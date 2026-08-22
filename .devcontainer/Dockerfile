# ============================================================
# Dockerfile: окружение UAV Digital Twin (ROS2 Jazzy + Gazebo Harmonic).
#
# База: osrf/ros:jazzy-simulation-noble — официальный образ OSRF,
# уже содержит ROS2 Jazzy + Gazebo Harmonic (gz-sim8) + ros_gz.
# ============================================================
FROM osrf/ros:jazzy-simulation-noble

ARG DEBIAN_FRONTEND=noninteractive

# --- Навигационный стек Nav2 + мосты Gazebo Harmonic (ros_gz_bridge) ---
# --- + MicroXRCEAgent (мост uORB <-> ROS2, БЕЗ MAVROS) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-bt-navigator \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-interfaces \
        ros-jazzy-ros-gz-image \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-octomap \
        ros-jazzy-octomap-msgs \
        ros-jazzy-octomap-server \
        ros-jazzy-nav2-voxel-grid \
        python3-pip \
        python3-venv \
        python3-dev \
        python3-setuptools \
        python3-wheel \
        python3-colcon-common-extensions \
        build-essential \
        cmake \
        ninja-build \
        xxd \
        git \
        libxml2-dev \
        libxml2-utils \
        file \
        unzip \
        zip \
        rsync \
    && rm -rf /var/lib/apt/lists/*

# --- Python-зависимости сборки PX4 (набор из официального px4-dev-base-noble) ---
RUN python3 -m pip install --break-system-packages --no-cache-dir \
        empy==3.3.4 \
        jinja2 \
        kconfiglib \
        packaging \
        pkgconfig \
        numpy \
        toml \
        pyyaml \
        pyserial \
        nunavut>=1.1.0 \
        pyulog \
        pyros-genmsg \
        jsonschema \
        future \
        lxml \
        psutil \
        pygments \
        pymavlink \
        requests \
        argcomplete \
        cerberus \
        sympy \
        symforce

WORKDIR /workspaces/ROS2_Nav2_PX4_Gazebo
