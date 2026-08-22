#!/usr/bin/env bash
# ============================================================
# Запуск миссии «восьмёрка» вокруг 5 столбиков.
#   - мир 5pillars.world (5 цилиндров h=3 м на круге R=3 м, без стены)
#   - mission_mode:=nav2 (следование за Nav2)
#   - takeoff_height:=2.0 (взлёт на 2 м; ниже ~1.5 м EKF/ywaw нестабилен)
#   - трек траектории (trajectory_trail) стартует в sim_full.launch.py
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

# Рендер через дискретную NVIDIA (GTX 1050)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

exec ros2 launch uav_bringup sim_full.launch.py \
    world:=5pillars.world \
    mission_mode:=nav2 \
    takeoff_height:=2.0
