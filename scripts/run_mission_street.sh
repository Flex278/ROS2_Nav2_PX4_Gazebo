#!/usr/bin/env bash
# ============================================================
# Запуск миссии «Патруль улицы» (street.world) в уже запущенной
# симуляции (bash scripts/launch_sim_headless.sh street.world).
# Сорсит ROS2 + workspace, чтобы rclpy был доступен (иначе
# "ModuleNotFoundError: No module named 'rclpy'").
#
# Примеры:
#   docker exec uav_mission bash -c 'cd /workspaces/ROS2_Nav2_PX4_Gazebo && bash scripts/run_mission_street.sh --speed 1.0'
#   docker exec uav_mission bash -c 'cd /workspaces/ROS2_Nav2_PX4_Gazebo && bash scripts/run_mission_street.sh --verify-only'
# ============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

exec python3 scripts/nav2_mission_street.py "$@"
