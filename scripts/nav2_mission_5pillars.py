#!/usr/bin/env python3
"""Миссия «восьмёрка» вокруг 5 столбиков (мир 5pillars.world).

Дрон взлетает на 2 м (takeoff_height:=2.0 при запуске) и облетает пять
цилиндрических ландмарков (высота 3 м, стоят по кругу радиусом 3 м) по
траектории «восьмёрка», проложенной через зазоры между столбами. Препятствия
обнаруживает камера глубины: depth_to_scan -> /scan -> costmap Nav2 ->
DWB огибает столбики, оказавшиеся на пути.

Запуск:
    # терминал 1 — симуляция + Nav2 + nav2_commander_node (взлёт на 2 м):
    bash scripts/run_sim_5pillars.sh

    # терминал 2 — миссия:
    python3 scripts/nav2_mission_5pillars.py

Аргументы:
    --speed        скорость движения [м/с] (по умолчанию 1.5)
    --timeout      таймаут одной цели [с] (по умолчанию 120)
    --frame        frame_id цели (по умолчанию 'odom')
    --waypoints    число точек траектории (по умолчанию 24)
    --verify-only  проверить зазор до столбов и выйти без запуска миссии
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

# ─── Геометрия «восьмёрки» вокруг 5 столбиков ──────────────────────────────
# Столбики стоят на круге радиусом 3 м (углы 0°, 72°, 144°, 216°, 288°).
# Лемниската с лепестками вдоль X принципиально несовместима с такой
# расстановкой: её левая петля пересекает кольцо ровно на столбах 144°/216°,
# поэтому цели оказывались внутри зоны отчуждения costmap. Траектория собрана
# вручную по зазорам между столбами:
#   * правая петля — дуга радиусом 2.0 м вокруг столба 0° (3,0), вход/выход
#     через зазоры 324° и 36°;
#   * левая петля — узкая линза вдоль -X через зазор 180° (между столбами
#     144°/216° коридор ~3.5 м).
# Мин. зазор до центра любого столба по всем точкам и сегментам >= 1.5 м.
FIG8_WAYPOINTS = 24     # число точек траектории (после ресемплинга)

# ─── Столбики мира 5pillars.world ──────────────────────────────────────────
# Пять цилиндров (r=0.2 м, h=3 м) на круге R=3 м, углы 0°/72°/144°/216°/288°.
# Координаты центров (x, y) в odom/ENU — те же, что в SDF мира.
PILLARS = [
    (3.0, 0.0),
    (0.927, 2.853),
    (-2.427, 1.763),
    (-2.427, -1.763),
    (0.927, -2.853),
]

# Минимальный зазор до центра столба [м]. Цель ближе SAFE_DIST попадает внутрь
# зоны отчуждения costmap (robot_radius 0.35 + inflation_radius 1.0 = 1.35 м) и
# становится недостижимой: DWB осциллирует на границе («Failed to make progress»,
# таймаут), дрон теряет контроль и врезается в столб. 1.5 м даёт запас ~0.15 м.
SAFE_DIST = 1.5


# Ключевые точки «восьмёрки» (x, y) в odom/ENU. Порядок: старт из (0,0),
# правая петля (вокруг столба 0°), переход через центр, левая петля (через
# зазор 180°), возврат на стартовую площадку.
_FIG8_KEYS: list[tuple[float, float]] = [
    (0.0, 0.0),
    (1.2, -0.9),
    (2.382, -1.902),      # зазор 324° — вход правой петли
    (4.0, -1.732),        # дуга вокруг столба 0°
    (5.0, 0.0),           # вершина правой петли (2.0 м от столба 0°)
    (4.0, 1.732),
    (2.382, 1.902),       # зазор 36° — выход правой петли
    (1.0, 1.0),
    (-1.0, -0.4),         # переход к левой петле
    (-2.0, -0.3),
    (-2.5, -0.2),
    (-3.0, -0.3),
    (-3.5, -0.6),
    (-4.0, -1.0),
    (-4.6, -0.6),
    (-5.0, 0.0),          # вершина левой петли
    (-4.6, 0.6),
    (-4.0, 1.0),
    (-3.5, 0.6),
    (-3.0, 0.3),
    (-2.5, 0.2),
    (-2.0, 0.3),
    (-1.0, 0.4),
    (0.0, 0.0),           # возврат на стартовую площадку
]


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


def generate_pillar_figure_eight(
        n: int = FIG8_WAYPOINTS) -> list[tuple[float, float, float]]:
    """Точки (x, y, yaw) «восьмёрки», огибающей 5 столбиков без сближения с ними."""
    pts = _resample_keys(_FIG8_KEYS, n)
    return [(x, y, 0.0) for x, y in pts]


def verify_clearance(
        waypoints: list[tuple[float, float, float]],
        pillars: list[tuple[float, float]] | None = None,
        min_dist: float = SAFE_DIST,
        samples: int = 25,
) -> tuple[float, float, tuple[int, int] | None]:
    """Минимальный зазор до столбов по точкам и по сегментам траектории."""
    pillars = pillars if pillars is not None else PILLARS

    def dmin(x: float, y: float) -> float:
        return min(math.hypot(x - px, y - py) for px, py in pillars)

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


def ensure_pillar_clearance(
        waypoints: list[tuple[float, float, float]],
        min_dist: float = SAFE_DIST,
        pillars: list[tuple[float, float]] | None = None,
) -> list[tuple[float, float, float]]:
    """Отодвинуть цели, оказавшиеся слишком близко к столбам.

    «Восьмёрка» пересекает кольцо столбов (R=3 м) в четырёх точках; при
    амплитуде 4.0 часть целей попадает внутрь зоны отчуждения costmap и
    становится недостижимой. Для каждой такой точки сдвигаем её радиально
    от центра ближайшего столба наружу до расстояния min_dist — форма «8»
    сохраняется, но гарантируется зазор до препятствия.
    """
    pillars = pillars if pillars is not None else PILLARS
    result: list[tuple[float, float, float]] = []
    for (x, y, yaw) in waypoints:
        nearest = min(pillars, key=lambda p: (x - p[0]) ** 2 + (y - p[1]) ** 2)
        dx, dy = x - nearest[0], y - nearest[1]
        dist = math.hypot(dx, dy)
        if dist < min_dist:
            if dist < 1e-9:
                # Цель ровно в центре столба — отодвигаем строго по +X.
                dx, dy, dist = 1.0, 0.0, 1.0
            x = nearest[0] + dx / dist * min_dist
            y = nearest[1] + dy / dist * min_dist
        result.append((x, y, yaw))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Миссия «восьмёрка» вокруг 5 столбиков')
    parser.add_argument('--speed', type=float, default=1.5,
                        help='Скорость движения [м/с]')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Таймаут одной цели [с]')
    parser.add_argument('--frame', default='odom',
                        help='frame_id цели (odom, т.к. map→odom нет)')
    parser.add_argument('--waypoints', type=int, default=FIG8_WAYPOINTS,
                        help='Число точек траектории (ресемплинг ломаной)')
    parser.add_argument('--verify-only', action='store_true',
                        help='Проверить зазор до столбов и выйти без запуска ROS')
    args = parser.parse_args()

    # Вывод print() идёт в stdout (буферизуется при редиректе в файл и
    # выгружался только при выходе — список целей оказывался в конце лога).
    # Включаем построчный сброс, чтобы заголовок и список целей шли первыми.
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(line_buffering=True)

    waypoints = generate_pillar_figure_eight(args.waypoints)
    waypoints = ensure_pillar_clearance(waypoints, min_dist=SAFE_DIST)
    wmin, segmin, worst = verify_clearance(waypoints)

    print(f'Миссия «восьмёрка»: {len(waypoints)} целей, '
          f'скорость {args.speed} м/с')
    print(f'  мин. зазор до столбов: точки {wmin:.2f} м, '
          f'сегменты {segmin:.2f} м'
          + (f' (сегмент {worst[0]}→{worst[1]})' if worst else ''))
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
