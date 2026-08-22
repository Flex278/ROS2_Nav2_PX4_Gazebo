#!/usr/bin/env bash
# ============================================================
# Запуск симуляции в headless-режиме (сервер gz sim -s)
# + Nav2 + nav2_commander_node + trajectory_trail.
# Мир задаётся первым аргументом (по умолчанию 5pillars.world),
# например:  ./scripts/launch_sim_headless.sh street.world
# GUI запускается отдельно скриптом launch_gz_gui_x0.sh на :0.
#
# БЕЗ NVIDIA-переменных: в контейнере нет libGLX_nvidia, а сервер
# headless и так не рендерит.
# ============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

# Собираем трек траектории, если бинарь отсутствует или исходник новее.
TRAIL_DIR="$PROJECT_ROOT/tools/trajectory_trail"
if [ ! -x "$TRAIL_DIR/trajectory_trail" ] || \
   [ "$TRAIL_DIR/trajectory_trail.cpp" -nt "$TRAIL_DIR/trajectory_trail" ]; then
    bash "$TRAIL_DIR/build.sh"
fi

WORLD="${1:-5pillars.world}"
echo "Запуск мира: ${WORLD}"

exec ros2 launch uav_bringup sim_full.launch.py \
    world:="${WORLD}" \
    mission_mode:=nav2 \
    takeoff_height:=3.0 \
    headless:=true
