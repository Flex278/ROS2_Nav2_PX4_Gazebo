"""Проекция карты глубины в лазерный скан (чистая логика, без rclpy)."""
from __future__ import annotations

import numpy as np


def depth_row_to_scan(
    depth_row: np.ndarray,
    fov_h: float,
    range_min: float,
    range_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Центральная строка карты глубины -> (angles, ranges).

    :param depth_row: (W,) float, дальности в метрах вдоль оптической оси
    :param fov_h: горизонтальный FOV камеры [рад]
    :param range_min: нижняя граница LaserScan [м]
    :param range_max: верхняя граница LaserScan [м]
    :return: (angles (W,), ranges (W,)); невалидные лучи -> inf
    """
    depth_row = np.asarray(depth_row, dtype=float)
    w = len(depth_row)
    angles = np.linspace(-fov_h / 2.0, fov_h / 2.0, w)
    valid = np.isfinite(depth_row) & (depth_row > 0.0)
    ranges = np.full(w, np.inf)
    if np.any(valid):
        cos_a = np.cos(angles[valid])
        ranges[valid] = np.clip(
            depth_row[valid] / np.where(cos_a == 0.0, 1.0, cos_a),
            range_min, range_max)
    return angles, ranges
