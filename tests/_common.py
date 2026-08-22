"""Общие помощники для локальных тестов (без pytest)."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# делаем импортируемыми пакеты uav_offboard / uav_perception / uav_sensors
for pkg in ('uav_offboard', 'uav_perception', 'uav_sensors'):
    p = os.path.join(ROOT, 'src', pkg)
    if os.path.isdir(p):
        sys.path.insert(0, p)
