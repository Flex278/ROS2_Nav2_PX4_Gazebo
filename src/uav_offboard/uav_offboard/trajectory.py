"""Генерация и дискретизация траекторий полёта.

Все координаты — локальная ENU-рамка (x — север, y — восток, z — вверх).
Перевод в NED (z -> -z) выполняется на границе публикации в PX4.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import comb
from typing import Sequence

import numpy as np


@dataclass
class Trajectory:
    """Дискретная траектория: моменты времени, позиции и скорости."""

    times: np.ndarray          # (M,) секунды
    positions: np.ndarray      # (M, 3) метры
    velocities: np.ndarray     # (M, 3) м/с

    def sample(self, t: float) -> np.ndarray:
        """Позиция в момент времени t (ближайший сэмпл, без экстраполяции)."""
        idx = int(np.clip(np.searchsorted(self.times, t), 0, len(self.times) - 1))
        return self.positions[idx]


def bezier_curve(
    points: Sequence[np.ndarray], num_samples: int = 100
) -> list[np.ndarray]:
    """Кривая Безье по опорным точкам.

    :param points: опорные точки (np.ndarray размерности 3)
    :param num_samples: количество точек дискретизации
    :return: список точек кривой
    """
    points = [np.asarray(p, dtype=float) for p in points]
    n = len(points) - 1
    t = np.linspace(0.0, 1.0, num_samples)
    curve: list[np.ndarray] = []
    for ti in t:
        pt = np.zeros(3)
        for i, p in enumerate(points):
            bern = comb(n, i) * (ti ** i) * ((1.0 - ti) ** (n - i))
            pt += bern * p
        curve.append(pt)
    return curve


def constant_speed_trajectory(
    waypoints: Sequence[np.ndarray],
    speed: float,
    dt: float,
    hover_time: float = 1.0,
) -> Trajectory:
    """Траектория с постоянной скоростью между точками и зависанием в каждой.

    :param waypoints: последовательность точек ENU (x, y, z)
    :param speed: модуль скорости на перегонах [м/с]
    :param dt: шаг дискретизации [с]
    :param hover_time: время зависания в каждой точке [с]
    """
    waypoints = [np.asarray(w, dtype=float) for w in waypoints]
    if not waypoints:
        raise ValueError('Список waypoints пуст')

    times: list[float] = []
    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []

    def add(pt: np.ndarray, vel: np.ndarray, t: float) -> None:
        positions.append(pt)
        velocities.append(vel)
        times.append(t)

    t = 0.0
    add(waypoints[0], np.zeros(3), t)
    t += dt

    for i in range(1, len(waypoints)):
        a = waypoints[i - 1]
        b = waypoints[i]
        seg = b - a
        dist = float(np.linalg.norm(seg))
        if dist > 1e-6:
            vel = seg / dist * speed
            n_steps = max(1, int(round(dist / speed / dt)))
            for k in range(1, n_steps + 1):
                add(a + seg * (k / n_steps), vel, t)
                t += dt
        n_hover = max(1, int(round(hover_time / dt)))
        for _ in range(n_hover):
            add(b, np.zeros(3), t)
            t += dt

    return Trajectory(np.array(times), np.array(positions), np.array(velocities))
