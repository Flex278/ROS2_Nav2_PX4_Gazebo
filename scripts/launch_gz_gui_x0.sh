#!/usr/bin/env bash
# ============================================================
# Перезапуск Gazebo GUI на реальном дисплее хоста :0.
# Подключается к уже работающему серверу gz sim (любой мир: 5pillars/street).
#   - DISPLAY=:0 (реальный рабочий стол, виден пользователю)
#   - программный рендер Mesa/llvmpipe: в контейнере нет NVIDIA GL
#     (libGLX_nvidia), поэтому __GLX_VENDOR_LIBRARY_NAME=nvidia роняет
#     ogre2 при инициализации. Без override GLVND выбирает Mesa.
# ============================================================
set -e

# --- Окружение ROS2 + Gazebo (vendor-пакеты Jazzy) ---
source /opt/ros/jazzy/setup.bash
source /workspaces/ROS2_Nav2_PX4_Gazebo/install/setup.bash

# --- Пути к моделям и мирам (те же, что у запущенного сервера) ---
export GZ_SIM_RESOURCE_PATH="/workspaces/ROS2_Nav2_PX4_Gazebo/install/uav_gazebo/share/uav_gazebo/models:/workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot/Tools/simulation/gz/models:/opt/ros/jazzy/share:/workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot/Tools/simulation/gz/worlds"

# --- Реальный дисплей хоста ---
export DISPLAY=:0

# --- Тёмный фон: используем собственную GUI-конфигурацию (background_color) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec gz sim -g -v 3 --render-engine ogre2 --gui-config "$SCRIPT_DIR/gz_gui_dark.config"
