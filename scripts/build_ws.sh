#!/usr/bin/env bash
# ============================================================
# Сборка ROS2 workspace (colcon).
# ============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --event-handlers console_direct+
