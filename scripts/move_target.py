#!/usr/bin/env python3
"""Движущаяся цель для Gazebo: телепортирует модели по траекториям (set_pose).

Gazebo 8.11 (Harmonic) убрал `gz model -x/-y/-z/-Y`, поэтому используется сервис
UserCommands через CLI `gz service`:

    gz service -s /world/<world>/set_pose \
        --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
        --req 'name: "<model>", position: {x: 1.0, y: 2.0, z: 0.0}'

Запуск (внутри контейнера, при поднятой симуляции street):
    python3 scripts/move_target.py --world street --model pedestrian \
        --mode pingpong --x0 10 --y0 4.5 --x1 10 --y1 -4.5 --height 0.2 --period 20

Траектории:
    circle   — окружность вокруг (--cx, --cy) радиусом --radius;
    pingpong — линейно туда-обратно между (--x0,--y0) и (--x1,--y1).

--model принимает список через запятую: все модели следуют одной траектории
(с одним --height и --yaw). `--dry-run` печатает команды без обращения к сервису.

Примечание: модели объявлены <static>true</static>. set_pose обновляет
Pose-компонент и для static-моделей, но если цель не сдвигается — уберите static
у модели и подберите --height так, чтобы объект стоял на земле
(тротуар 0.2 м, дорога 0.04 м).
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import time


# ---------- Чистая геометрия траекторий (тестируется без ROS/Gazebo) ----------

def circle_xy(t: float, radius: float, cx: float, cy: float,
              period: float) -> tuple[float, float]:
    """Позиция на окружности в момент t (t=0 -> точка (cx+radius, cy))."""
    ang = 2.0 * math.pi * t / period
    return cx + radius * math.cos(ang), cy + radius * math.sin(ang)


def pingpong_xy(t: float, p0: tuple[float, float], p1: tuple[float, float],
                period: float) -> tuple[float, float]:
    """Позиция при движении туда-обратно p0->p1->p0 за `period` секунд."""
    phase = (t % period) / period  # 0..1
    if phase < 0.5:
        frac = phase * 2.0
    else:
        frac = 2.0 * (1.0 - phase)
    return p0[0] + (p1[0] - p0[0]) * frac, p0[1] + (p1[1] - p0[1]) * frac


def trajectory_xy(mode: str, t: float, args: argparse.Namespace) -> tuple[float, float]:
    """Вычислить (x, y) для заданного режима траектории."""
    if mode == 'circle':
        return circle_xy(t, args.radius, args.cx, args.cy, args.period)
    if mode == 'pingpong':
        return pingpong_xy(t, (args.x0, args.y0), (args.x1, args.y1), args.period)
    raise ValueError(f'неизвестный режим: {mode}')


# ---------- Телепорт через gz service set_pose ----------

def gz_bin() -> str:
    """Путь к бинарю `gz` (обычно нужен source /opt/ros/jazzy/setup.bash)."""
    exe = shutil.which('gz')
    if exe:
        return exe
    vendor = '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
    return vendor if os.path.exists(vendor) else 'gz'


def pose_req(model: str, x: float, y: float, z: float, yaw: float = 0.0) -> str:
    """Текст gz.msgs.Pose (protobuf text) для set_pose."""
    if yaw:
        hw = yaw / 2.0
        orient = (f', orientation: {{x: 0, y: 0, '
                  f'z: {math.sin(hw):.6f}, w: {math.cos(hw):.6f}}}')
    else:
        orient = ''
    return (f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, '
            f'z: {z:.4f}}}{orient}')


def set_pose(model: str, x: float, y: float, z: float, yaw: float, world: str,
             dry_run: bool = False) -> None:
    """Переместить модель через gz service set_pose (телепорт, без физики)."""
    cmd = [gz_bin(), 'service', '-s', f'/world/{world}/set_pose',
           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '1000', '--req', pose_req(model, x, y, z, yaw)]
    if dry_run:
        print(' '.join(cmd))
        return
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f'[move_target] ОШИБКА запуска gz service: {exc}', file=sys.stderr)
        return
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout).decode(errors='replace').strip()
        print(f'[move_target] ОШИБКА gz service ({model}): {err}', file=sys.stderr)


def main() -> None:
    ap = argparse.ArgumentParser(description='Движущаяся цель для Gazebo (set_pose)')
    ap.add_argument('--world', default='street', help='имя мира (default: street)')
    ap.add_argument('--model', default='pedestrian',
                    help='имя модели или список через запятую')
    ap.add_argument('--mode', choices=['circle', 'pingpong'], default='pingpong')
    ap.add_argument('--radius', type=float, default=3.0, help='[circle] радиус [м]')
    ap.add_argument('--cx', type=float, default=0.0, help='[circle] X центра')
    ap.add_argument('--cy', type=float, default=0.0, help='[circle] Y центра')
    ap.add_argument('--x0', type=float, default=10.0, help='[pingpong] X точки A')
    ap.add_argument('--y0', type=float, default=4.5, help='[pingpong] Y точки A')
    ap.add_argument('--x1', type=float, default=10.0, help='[pingpong] X точки B')
    ap.add_argument('--y1', type=float, default=-4.5, help='[pingpong] Y точки B')
    ap.add_argument('--height', type=float, default=0.2,
                    help='Z модели [м] (тротуар 0.2, дорога 0.04)')
    ap.add_argument('--yaw', type=float, default=0.0, help='рыскание модели [рад]')
    ap.add_argument('--period', type=float, default=20.0,
                    help='сек на полный цикл траектории')
    ap.add_argument('--rate', type=float, default=10.0, help='Гц обновления позы')
    ap.add_argument('--dry-run', action='store_true', help='только печатать команды')
    args = ap.parse_args()

    models = [m.strip() for m in args.model.split(',') if m.strip()]
    dt = 1.0 / args.rate
    print(f'[move_target] world={args.world} models={models} mode={args.mode} '
          f'period={args.period}s ({args.rate} Гц). Ctrl+C — стоп.')
    t0 = time.time()
    try:
        while True:
            x, y = trajectory_xy(args.mode, time.time() - t0, args)
            for model in models:
                set_pose(model, x, y, args.height, args.yaw, args.world,
                         dry_run=args.dry_run)
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\n[move_target] остановлено.')


if __name__ == '__main__':
    main()
