"""Живое демо software-пайплайна UAV (без ROS2 / PX4 / Gazebo).

Прогоняет в терминале реальную логику:
  1. Безье-траектория
  2. MPC-контроллер (замкнутый контур по траектории)
  3. Автомат миссии (arm -> takeoff -> hover -> waypoints -> land -> disarm)
  4. depth -> laser scan (проекция центральной строки)
  5. Детекция H-маркера на синтезированном кадре

Запуск:  python3 scripts/demo_pipeline.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('uav_offboard', 'uav_perception', 'uav_sensors'):
    p = os.path.join(ROOT, 'src', pkg)
    if os.path.isdir(p):
        sys.path.insert(0, p)

from uav_offboard.mission_controller import (  # noqa: E402
    Command, MissionController, NAV_STATE_AUTO_LAND, NAV_STATE_OFFBOARD,
)
from uav_offboard.mpc_controller import MPCController  # noqa: E402
from uav_offboard.trajectory import bezier_curve  # noqa: E402
from uav_perception.detection import detect_h_marker  # noqa: E402
from uav_sensors.scan_projection import depth_row_to_scan  # noqa: E402


def section(title: str) -> None:
    print('\n' + '=' * 74)
    print('  ' + title)
    print('=' * 74)


# --------------------------------------------------------------------------- #
section('1. Безье-траектория: облёт посадочной площадки (H-маркер)')
# --------------------------------------------------------------------------- #
pts = [np.array([0.0, 0.0, 2.0]),
       np.array([2.0, 1.0, 2.5]),
       np.array([4.0, 0.0, 2.0]),
       np.array([2.0, -1.0, 1.8])]
curve = bezier_curve(pts, num_samples=7)
print('Опорные точки:')
for i, p in enumerate(pts):
    print(f'  P{i}: ({p[0]:+.1f}, {p[1]:+.1f}, {p[2]:.1f})')
print('Кривая Безье (7 сэмплов):')
for i, p in enumerate(curve):
    print(f'  t={i / 6:.3f}  ->  ({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:.2f})')


# --------------------------------------------------------------------------- #
section('2. MPC: замкнутый контур слежения (двойной интегратор)')
# --------------------------------------------------------------------------- #
mpc = MPCController(horizon=10, dt=0.1, max_acc=2.0)
state = np.zeros(6)  # x,y,z,vx,vy,vz
targets = [np.array([0.0, 0.0, 2.0]),
           np.array([3.0, 2.0, 2.0]),
           np.array([3.0, -2.0, 2.0]),
           np.array([0.0, 0.0, 2.0])]
dt = 0.1
print(f'Старт из ({state[0]}, {state[1]}, {state[2]}), horizon={mpc.horizon}, '
      f'dt={dt}')
for ti, target in enumerate(targets):
    steps = 0
    while True:
        steps += 1
        u = mpc.compute_control(state, target)
        state[3:] += u * dt
        state[:3] += state[3:] * dt
        err = float(np.linalg.norm(state[:3] - target))
        if steps in (1, 10) or err < 0.1 or steps >= 200:
            print(f'  цель {ti}: ({target[0]:+.1f},{target[1]:+.1f},{target[2]:.1f})'
                  f' | шаг {steps:3d} | поз=({state[0]:+.2f},{state[1]:+.2f},'
                  f'{state[2]:.2f}) | a=({u[0]:+.2f},{u[1]:+.2f},{u[2]:+.2f}) '
                  f'| err={err:.3f}')
        if err < 0.1 or steps >= 200:
            break



# --------------------------------------------------------------------------- #
section('3. Автомат миссии: arm -> takeoff -> waypoints -> land -> disarm')
# --------------------------------------------------------------------------- #
waypoints = [[3.0, 2.0, 2.0], [3.0, -2.0, 2.0], [-2.0, 0.0, 2.0]]
mc = MissionController(takeoff_height=2.0, hover_time=2.0,
                       waypoints=waypoints, dt=0.1)
pos = np.zeros(3)
armed = False
nav_state = 0
now = 0.0
dt = 0.1
prev_state = None
print(f'Waypoints: {waypoints}, takeoff_height={mc.takeoff_height}m')
while not mc.finished and now < 120.0:
    res = mc.step(armed, nav_state, pos, now)

    # «физика»: применяем команды и двигаем дрон к setpoint
    for c in res.commands:
        if c is Command.ARM:
            armed = True
        elif c is Command.DISARM:
            armed = False
        elif c is Command.SET_OFFBOARD:
            nav_state = NAV_STATE_OFFBOARD
        elif c is Command.LAND:
            nav_state = NAV_STATE_AUTO_LAND

    if nav_state == NAV_STATE_AUTO_LAND:      # автопосадка PX4
        pos[2] = max(0.0, pos[2] - 0.5 * dt)
    elif res.setpoint is not None:
        d = res.setpoint - pos
        dist = float(np.linalg.norm(d))
        if dist > 1e-9:
            step_mv = min(dist, 1.0 * dt)
            pos = pos + d / dist * step_mv

    # печать: переходы состояний + позиция раз в секунду
    if mc.state is not prev_state:
        cmds = [c.name for c in res.commands]
        print(f'  >> {prev_state.name if prev_state else "-":>10}'
              f' -> {mc.state.name:<10}  cmd={cmds or "-"}')
        prev_state = mc.state
    if int(round(now * 2)) % 20 == 0:  # раз в 1.0 c
        sp = res.setpoint
        sp_s = (f'({sp[0]:+.2f},{sp[1]:+.2f},{sp[2]:.2f})') if sp is not None else '-'
        print(f'  t={now:5.1f}s  state={mc.state.name:<10}'
              f' pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f})  setpoint={sp_s}')

    now += dt

print(f'Итог: mission {"ВЫПОЛНЕНА" if mc.finished else "ПРЕРВАНА"} за {now:.1f}s')


# --------------------------------------------------------------------------- #
section('4. depth -> /scan: проекция центральной строки глубины')
# --------------------------------------------------------------------------- #
# Синтетическая строка глубины 16px: стена справа (2м), открытое небо слева
row = np.array([np.inf] * 6 + [4.0, 3.0, 2.5, 2.0] + [np.inf] * 6)
angles, ranges = depth_row_to_scan(row, fov_h=np.deg2rad(87.0),
                                   range_min=0.2, range_max=10.0)
print('depth_row (метры):', np.where(np.isinf(row), 'inf', row.astype(str)))
print(f'angles: {np.rad2deg(angles[0]):+.1f} .. {np.rad2deg(angles[-1]):+.1f} град '
      f'({len(angles)} лучей)')
print('ranges:', np.round(ranges, 2))


# --------------------------------------------------------------------------- #
section('5. Детекция H-маркера на синтезированном кадре (OpenCV)')
# --------------------------------------------------------------------------- #
import cv2  # noqa: E402

img = np.full((200, 200), 255, np.uint8)  # белый фон
# рисуем тёмную букву H: две ножки + перекладина
cv2.rectangle(img, (60, 40), (85, 160), 0, -1)     # левая ножка
cv2.rectangle(img, (115, 40), (140, 160), 0, -1)   # правая ножка
cv2.rectangle(img, (60, 85), (140, 110), 0, -1)    # перекладина
det = detect_h_marker(img)
if det is not None:
    print(f'ОБНАРУЖЕН: {det.class_name}  conf={det.confidence:.2f}  '
          f'bbox={det.bbox}')
else:
    print('НЕ обнаружен (ожидался H-маркер)')

print('\nГОТОВО: все 5 модулей пайплайна отработали.')
