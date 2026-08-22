#!/usr/bin/env bash
# ============================================================
# Запуск полной симуляции: PX4 SITL + Gazebo Harmonic + мосты + Nav2 + CV.
# ============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

# Рендер через дискретную NVIDIA (GTX 1050)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

exec ros2 launch uav_bringup sim_full.launch.py
