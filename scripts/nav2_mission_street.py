#!/usr/bin/env python3
"""Миссия «Патруль улицы» (мир street.world) — валидация Фазы A на реальной сцене.

Дрон взлетает на 3 м (takeoff_height:=3.0 при запуске) и летит «змейкой» вдоль
дороги (ось X) от старта (0,0) до дальней машины (x=25) и обратно. По пути:

  * Nav2 (depth_to_scan -> /scan -> costmap -> DWB) огибает ВЫСОКИЕ объекты
    (>=3 м), которые попадают в /scan на высоте полёта 3 м: 4 фонаря
    (x=0/20, y=±4.25), светофор (x=10, y=4.25), знак «стоп» (x=6, y=4.25).
  * CV (human_detector + target_localizer) детектит НИЗКИЕ объекты (<3 м):
    пешехода (x=10, y=3.7), машины car_blue (0,2.6) / car_green (15,-2.6) /
    car_red (25,2.6) и конусы у перехода. На высоте 3 м они под дроном, в
    /scan не попадают и остаются чистыми CV-целями.

Это недостающий хвост Фазы A: проверка полного стека (headless + GUI + Nav2 +
PX4) на мире street и знака Y/Z в /target/position на реальной сцене.

Запуск:
    # терминал 1 — симуляция + Nav2 + nav2_commander_node (взлёт на 3 м):
    bash scripts/launch_sim_headless.sh street.world
    bash scripts/launch_gz_gui_x0.sh

    # терминал 2 — миссия:
    python3 scripts/nav2_mission_street.py

Аргументы:
    --speed        скорость движения [м/с] (по умолчанию 1.0)
    --timeout      таймаут одной цели [с] (по умолчанию 120)
    --frame        frame_id цели (по умолчанию 'odom')
    --waypoints    число точек траектории (ресемплинг ломаной)
    --verify-only  проверить зазор до высоких препятствий и выйти без ROS
"""
from __future__ import annotations

import argparse
import math
import os
import sys

# scripts/ не является пакетом, добавляем его в sys.path для импорта
# MultiGoalClient из nav2_mission.py (импорт выполняется только при запуске
# миссии, чтобы флаг --verify-only работал без ROS).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ─── ВЫСОКИЕ препятствия мира street.world (видны в /scan на высоте 2 м) ────
# Центры (x, y) в odom/ENU — те же, что в SDF мира. Nav2 огибает их, поэтому
# маршрут держит зазор >= SAFE_DIST от каждого центра.
STREET_OBSTACLES = [
    (0.0, 4.25),     # street_light_1
    (20.0, 4.25),    # street_light_2
    (0.0, -4.25),    # street_light_3
    (20.0, -4.25),   # street_light_4
    (10.0, 4.25),    # traffic_light
    (6.0, 4.25),     # stop_sign
]

# ─── НИЗКИЕ CV-цели (справочно): их детектит CV, в /scan они НЕ попадают ─────
CV_TARGETS = {
    'car_blue': (0.0, 2.6),
    'car_green': (15.0, -2.6),
    'car_red': (25.0, 2.6),
    'pedestrian': (10.0, 3.7),
    'cone_1': (8.5, 2.5),
    'cone_2': (11.5, 2.5),
    'cone_3': (8.5, -2.5),
    'cone_4': (11.5, -2.5),
}

# Минимальный зазор до центра высокого препятствия [м]. Цель ближе SAFE_DIST
# попадает внутрь зоны отчуждения costmap (robot_radius 0.35 + inflation_radius
# 1.0 = 1.35 м) и становится недостижимой: DWB осциллирует на границе и дрон
# теряет контроль. 1.6 м даёт запас ~0.25 м (маршрут ниже держит >=2.0 м).
SAFE_DIST = 1.6

# ─── Ключевые точки маршрута (x, y) в odom/ENU ───────────────────────────────
# «Змейка» вдоль дороги: к каждой CV-цели дрон подходит со стороны -X (камера
# смотрит вперёд +X), поэтому цель попадает в передний FOV по мере приближения.
# Зазор до всех ВЫСОКИХ препятствий по точкам и сегментам >= 2.0 м (проверяется
# функцией verify_clearance, см. --verify-only).
_STREET_KEYS: list[tuple[float, float]] = [
    (0.0, 0.0),        # старт на дороге (car_blue справа, y=2.6)
    (-3.0, 1.8),       # заход за car_blue (0,2.6)
    (3.0, 1.8),        # пролёт мимо car_blue
    (7.0, 2.2),        # заход на переход (пешеход 10,3.7 + конусы y=±2.5)
    (13.0, 2.2),       # пролёт перехода (зебра x=10)
    (12.0, -1.8),      # переход на левую полосу к car_green (15,-2.6)
    (18.0, -1.8),      # пролёт мимо car_green
    (22.0, 1.8),       # переход к car_red (25,2.6)
    (28.0, 1.8),       # пролёт мимо car_red
    (0.0, 0.0),        # возврат на старт
]

PATROL_WAYPOINTS = 28   # число точек траектории (после ресемплинга)


def _resample_keys(keys: list[tuple[float, float]],
                   n: int) -> list[tuple[float, float]]:
    """Равномерно пересчитать N точек вдоль ломаной из ключевых точек."""
    if n <= len(keys):
        return list(keys)
    segs = [math.hypot(keys[i + 1][0] - keys[i][0],
                       keys[i + 1][1] - keys[i][1])
            for i in range(len(keys) - 1)]
    total = sum(segs)
    pts: list[tuple[float, float]] = []
    for i in range(n):
        target = total * i / n
        acc = 0.0
        for k, s in enumerate(segs):
            if acc + s >= target - 1e-9 or k == len(segs) - 1:
                f = 0.0 if s < 1e-12 else (target - acc) / s
                f = min(max(f, 0.0), 1.0)
                pts.append((keys[k][0] + (keys[k + 1][0] - keys[k][0]) * f,
                            keys[k][1] + (keys[k + 1][1] - keys[k][1]) * f))
                break
            acc += s
    pts[-1] = keys[-1]
    return pts


def generate_street_patrol(
        n: int = PATROL_WAYPOINTS) -> list[tuple[float, float, float]]:
    """Точки (x, y, yaw) патрульного маршрута вдоль улицы."""
    pts = _resample_keys(_STREET_KEYS, n)
    return [(x, y, 0.0) for x, y in pts]


def verify_clearance(
        waypoints: list[tuple[float, float, float]],
        obstacles: list[tuple[float, float]] | None = None,
        min_dist: float = SAFE_DIST,
        samples: int = 25,
) -> tuple[float, float, tuple[int, int] | None]:
    """Минимальный зазор до высоких препятствий по точкам и по сегментам."""
    obstacles = obstacles if obstacles is not None else STREET_OBSTACLES

    def dmin(x: float, y: float) -> float:
        return min(math.hypot(x - px, y - py) for px, py in obstacles)

    wmin = min(dmin(x, y) for x, y, _ in waypoints)
    segmin = float('inf')
    worst: tuple[int, int] | None = None
    for i in range(len(waypoints) - 1):
        x1, y1, _ = waypoints[i]
        x2, y2, _ = waypoints[i + 1]
        sm = min(dmin(x1 + (x2 - x1) * f, y1 + (y2 - y1) * f)
                 for f in (s / samples for s in range(samples + 1)))
        if sm < segmin:
            segmin = sm
            worst = (i + 1, i + 2)
    return wmin, segmin, worst


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Миссия «Патруль улицы» (мир street.world)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Скорость движения [м/с]')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Таймаут одной цели [с]')
    parser.add_argument('--frame', default='odom',
                        help='frame_id цели (odom, т.к. map→odom нет)')
    parser.add_argument('--waypoints', type=int, default=PATROL_WAYPOINTS,
                        help='Число точек траектории (ресемплинг ломаной)')
    parser.add_argument('--verify-only', action='store_true',
                        help='Проверить зазор до высоких препятствий и выйти без ROS')
    args = parser.parse_args()

    # Вывод print() идёт в stdout (буферизуется при редиректе в файл и
    # выгружался только при выходе — список целей оказывался в конце лога).
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(line_buffering=True)

    waypoints = generate_street_patrol(args.waypoints)
    wmin, segmin, worst = verify_clearance(waypoints)

    print(f'Миссия «Патруль улицы»: {len(waypoints)} целей, '
          f'скорость {args.speed} м/с')
    print(f'  мин. зазор до высоких препятствий: точки {wmin:.2f} м, '
          f'сегменты {segmin:.2f} м'
          + (f' (сегмент {worst[0]}→{worst[1]})' if worst else ''))
    print('  CV-цели (низкие, детектит CV):')
    for name, (tx, ty) in CV_TARGETS.items():
        print(f'    {name:11s} x={tx:+5.1f}  y={ty:+5.1f}')
    for i, (x, y, yaw) in enumerate(waypoints, 1):
        print(f'  {i:2d}. x={x:+6.2f}  y={y:+6.2f}  yaw={yaw:.1f}')

    if args.verify_only:
        if min(wmin, segmin) < SAFE_DIST:
            print(f'ОШИБКА: зазор < {SAFE_DIST} м '
                  f'(точки {wmin:.2f} м, сегменты {segmin:.2f} м)')
            raise SystemExit(1)
        print(f'OK: минимальный зазор {min(wmin, segmin):.2f} м '
              f'>= {SAFE_DIST} м')
        raise SystemExit(0)

    # Импортируем ROS-зависимости только при запуске миссии.
    import rclpy  # noqa: E402
    from nav2_mission import MultiGoalClient  # noqa: E402

    rclpy.init()
    node = MultiGoalClient(speed=args.speed, frame=args.frame)
    code = 1
    try:
        ok = node.run_mission(waypoints, goal_timeout=args.timeout)
        code = 0 if ok else 1
    except KeyboardInterrupt:
        print('\nПрервано пользователем.')
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
