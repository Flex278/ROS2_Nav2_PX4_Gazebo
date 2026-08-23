# ============================================================
# Dockerfile: самодостаточный образ UAV Digital Twin
# (ROS2 Jazzy + Gazebo Harmonic + PX4 SITL + MicroXRCEAgent +
#  workspace + venv + trajectory_trail)
#
# После docker build образ содержит ВСЁ для запуска миссии.
# Запуск миссии «улица» — 3 команды:
#   1. docker run -d --name uav_mission --network host --privileged --gpus all \
#        -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#        uav-dt:jazzy sleep infinity
#   2. docker exec uav_mission bash scripts/launch_sim_headless.sh street.world
#   3. docker exec uav_mission bash scripts/run_mission_street.sh
# ============================================================
FROM osrf/ros:jazzy-simulation-noble

ARG DEBIAN_FRONTEND=noninteractive

# ─── Системные зависимости (Nav2, ros_gz, инструменты сборки) ───
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

# ─── Python-зависимости сборки PX4 ───
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

# ─── Клонирование PX4-Autopilot (зафиксированный коммит) ───
ARG PX4_COMMIT=99c40407ffd7ac184e2d7b4b293f36f10fe561ef
ARG GZ_MODELS_COMMIT=d754381a1cecdd7f17050acd72bf5bf1327bced6

RUN git clone --depth 1 --branch v1.15.4 \
        https://github.com/PX4/PX4-Autopilot.git \
        /workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot \
    && cd /workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot \
    && git fetch --depth 1 origin ${PX4_COMMIT} \
    && git checkout ${PX4_COMMIT} \
    && git submodule update --init --recursive --depth 1 \
    && git fetch --tags --depth 1 origin \
    && git submodule foreach --recursive \
        'git fetch --tags --depth 1 origin 2>/dev/null || true' \
    && cd Tools/simulation/gz \
    && git fetch --depth 1 origin ${GZ_MODELS_COMMIT} \
    && git checkout ${GZ_MODELS_COMMIT} \
    && cd /workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot \
    && git config --global --add safe.directory '*' || true

# ─── Применяем патчи к PX4 ───
COPY patches/px4-autopilot.patch /tmp/px4-autopilot.patch
COPY patches/px4-gazebo-models.patch /tmp/px4-gazebo-models.patch

RUN cd /workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot \
    && git apply /tmp/px4-autopilot.patch \
    && cd Tools/simulation/gz \
    && git apply /tmp/px4-gazebo-models.patch \
    && rm /tmp/px4-autopilot.patch /tmp/px4-gazebo-models.patch

# ─── Сборка PX4 SITL (make px4_sitl_default) ───
RUN . /opt/ros/jazzy/setup.sh \
    && export PKG_CONFIG_PATH="$(find /opt/ros/jazzy/opt \
        -name pkgconfig -type d 2>/dev/null | tr '\n' ':')${PKG_CONFIG_PATH:-}" \
    && cd /workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot \
    && make px4_sitl_default

# ─── Сборка Micro XRCE DDS Agent (мост uORB ↔ ROS2) ───
ARG AGENT_VERSION=v2.4.2
RUN git clone --branch ${AGENT_VERSION} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        /opt/Micro-XRCE-DDS-Agent \
    && cd /opt/Micro-XRCE-DDS-Agent \
    && sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' \
        CMakeLists.txt \
    && sed -i 's/-DSHM_TRANSPORT_DEFAULT:BOOL=OFF/-DSHM_TRANSPORT_DEFAULT:BOOL=OFF\n                -DFASTDDS_STATISTICS:BOOL=OFF/' \
        cmake/SuperBuild.cmake \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release \
          -DUAGENT_BUILD_EXECUTABLE=ON \
          -DUAGENT_FAST_PROFILE=ON \
          -DUAGENT_CED_PROFILE=OFF \
          -DUAGENT_P2P_PROFILE=OFF \
          -DUAGENT_DISCOVERY_PROFILE=OFF \
          .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig /usr/local/lib

# ─── Копируем исходники проекта (PX4-Autopilot исключён .dockerignore) ───
COPY . .

# ─── Сборка ROS2 workspace (colcon) ───
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+

# ─── Python venv для CV (opencv, ultralytics, torch) ───
# Используем venv/bin/pip напрямую (без source activate — RUN работает в /bin/sh).
RUN python3 -m venv /workspaces/ROS2_Nav2_PX4_Gazebo/venv \
    && /workspaces/ROS2_Nav2_PX4_Gazebo/venv/bin/pip install --upgrade pip \
    && /workspaces/ROS2_Nav2_PX4_Gazebo/venv/bin/pip install opencv-python \
    && /workspaces/ROS2_Nav2_PX4_Gazebo/venv/bin/pip install ultralytics \
    && /workspaces/ROS2_Nav2_PX4_Gazebo/venv/bin/pip install torch==2.13.0 \
        --index-url https://download.pytorch.org/whl/cu130

# ─── Сборка trajectory_trail ───
RUN . /opt/ros/jazzy/setup.sh \
    && cd /workspaces/ROS2_Nav2_PX4_Gazebo/tools/trajectory_trail \
    && GZ_PREFIX="/opt/ros/jazzy/opt" \
    && export PKG_CONFIG_PATH="$(find "$GZ_PREFIX" -type d \
        -name pkgconfig 2>/dev/null | tr '\n' ':')${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}" \
    && RPATH="$(find "$GZ_PREFIX" -maxdepth 3 -type d -name lib 2>/dev/null \
        | tr '\n' ':')" \
    && RPATH="${RPATH%:}" \
    && g++ -std=c++17 -O2 trajectory_trail.cpp -o trajectory_trail \
        -Wl,--disable-new-dtags \
        -Wl,-rpath,"$RPATH" \
        $(pkg-config --cflags --libs gz-transport13 gz-msgs10 gz-math7)

# ─── Очистка кэша pip ───
RUN rm -rf /root/.cache/pip

CMD ["sleep", "infinity"]
