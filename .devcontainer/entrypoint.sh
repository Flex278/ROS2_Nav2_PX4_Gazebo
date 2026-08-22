#!/usr/bin/env bash
# ============================================================
# Инициализация окружения внутри Dev Container.
# ============================================================
set -e

# 1. ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

# 2. Рабочее пространство (если уже собрано)
if [ -f /workspaces/ROS2_Nav2_PX4_Gazebo/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /workspaces/ROS2_Nav2_PX4_Gazebo/install/setup.bash
fi

# 3. Форсируем рендер через дискретную NVIDIA (GTX 1050)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

exec "$@"
