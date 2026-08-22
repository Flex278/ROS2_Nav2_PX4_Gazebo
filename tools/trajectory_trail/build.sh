#!/usr/bin/env bash
# ============================================================
# Сборка trajectory_trail (gz-transport13 + gz-msgs10 + gz-math7).
# Используются vendored-библиотеки Gazebo Harmonic из ROS2 Jazzy.
# ============================================================
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

GZ_PREFIX="/opt/ros/jazzy/opt"
# Подключаем ВСЕ vendored-пакеты Gazebo (transport/msgs/math + зависимости).
export PKG_CONFIG_PATH="$(find "$GZ_PREFIX" -type d -name pkgconfig 2>/dev/null | tr '\n' ':')${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

CXX=${CXX:-g++}
# rpath на все vendor-каталоги lib — бинарь найдёт библиотеки без
# необходимости source ROS (полезно и для запуска через ros2 launch).
RPATH="$(find "$GZ_PREFIX" -maxdepth 3 -type d -name lib 2>/dev/null | tr '\n' ':')"
RPATH="${RPATH%:}"
$CXX -std=c++17 -O2 trajectory_trail.cpp -o trajectory_trail \
  -Wl,--disable-new-dtags \
  -Wl,-rpath,"$RPATH" \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10 gz-math7)

echo "Собрано: $(pwd)/trajectory_trail"
