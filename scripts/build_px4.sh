#!/usr/bin/env bash
# ============================================================
# build_px4.sh — предсборка PX4 SITL внутри контейнера.
# ROS2 (Jazzy) подключается для доступа к вендоренному Gazebo
# Harmonic (gz-sim8 из gz_sim_vendor) через PKG_CONFIG_PATH.
# ============================================================
set -e

# ROS2 Jazzy + вендоренный Gazebo Harmonic (gz-sim8)
source /opt/ros/jazzy/setup.bash

# Пробрасываем pkg-config пути вендоренного Gazebo (иначе PX4 gz-плагин не соберётся)
export PKG_CONFIG_PATH="$(find /opt/ros/jazzy/opt -name pkgconfig -type d 2>/dev/null | tr '\n' ':')${PKG_CONFIG_PATH:-}"

PX4_DIR="${1:-/workspaces/ROS2_Nav2_PX4_Gazebo/PX4-Autopilot}"
cd "$PX4_DIR"

# Клон принадлежит хосту (uid 1000), а контейнер работает под root.
# Без этого git откажется читать репозиторий ("dubious ownership"),
# и сборка PX4 упадёт на определении версии (string REPLACE / git describe).
git config --global --add safe.directory "$PX4_DIR" || true
git config --global --add safe.directory '*' || true

echo ">>> gz pkg-config пути:"
echo "$PKG_CONFIG_PATH" | tr ':' '\n' | grep -i gz || true
echo ">>> make px4_sitl_default"
make px4_sitl_default
