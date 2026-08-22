"""Локализация цели: bbox -> 3D через карту глубины (чистая логика, без rclpy).

Принцип (проект OakD-Lite в x500_depth, см. model.sdf):
  * RGB-камера  IMX214:      640x360, horizontal_fov=1.204 рад (~69°).
  * Глубина     StereoOV7251: 320x240, horizontal_fov=1.274 рад (~73°),
    формат R_FLOAT32 -> ROS 32FC1, значение = дальность ВДОЛЬ ОПТИЧЕСКОЙ ОСИ
    (Z-depth, как у Gazebo depth_camera; depth_to_scan делит её на cos угла).
  * Оба сенсора совпадают по позе/ориентации (0.01233 -0.03 .01878, без
    поворота), отличаются только FOV и разрешением. Поэтому луч, восстановленный
    из RGB-пикселя, пересчитывается в пиксель карты глубины по углам.

Конвенция кадра камеры: X вперёд, Y влево, Z вверх (совпадает с base_link, без
поворота; static_transform_publisher camera_*_tf в sim_full.launch.py).
Знаки согласованы с depth_to_scan: изображение Gazebo зеркально, поэтому
правый край кадра = левая сторона мира = +Y. Формула: tan(angle) = (u-cx)/fx,
angle>0 соответствует +Y (влево); по вертикали tan(pitch) = (cy-v)/fy,
pitch>0 соответствует +Z (вверх).
"""
from __future__ import annotations

import numpy as np

# Параметры камер OakD-Lite (PX4-Autopilot/.../models/OakD-Lite/model.sdf).
RGB_WIDTH = 640
RGB_HEIGHT = 360
RGB_HFOV = 1.204  # рад

DEPTH_WIDTH = 320
DEPTH_HEIGHT = 240
DEPTH_HFOV = 1.274  # рад


def vertical_fov(hfov: float, width: int, height: int) -> float:
    """Вертикальный FOV по горизонтальному и соотношению сторон [рад]."""
    return 2.0 * np.arctan(np.tan(hfov / 2.0) * height / width)


def intrinsics(width: int, height: int, hfov: float) -> tuple[float, float, float, float]:
    """(fx, fy, cx, cy) камеры без дисторсии по разрешению и горизонтальному FOV."""
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    fy = (height / 2.0) / np.tan(vertical_fov(hfov, width, height) / 2.0)
    return fx, fy, width / 2.0, height / 2.0


def bbox_center(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    """Центр рамки (x_min, y_min, x_max, y_max) -> (u, v) [пиксели]."""
    x_min, y_min, x_max, y_max = bbox
    return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0


def bbox_to_position(
    bbox: tuple[int, int, int, int],
    depth: np.ndarray,
    rgb_k: tuple[float, float, float, float] | None = None,
    depth_k: tuple[float, float, float, float] | None = None,
    window: int = 3,
) -> tuple[float, float, float] | None:
    """Рамка в RGB-пикселях + карта глубины -> (X, Y, Z) в кадре камеры [м].

    X — вперёд (вдоль оптической оси), Y — влево, Z — вверх.

    :param bbox: (x_min, y_min, x_max, y_max) в пикселях RGB-кадра.
    :param depth: (H, W) float, дальности вдоль оптической оси [м].
    :param rgb_k: (fx, fy, cx, cy) RGB-камеры (по умолчанию — из констант).
    :param depth_k: (fx, fy, cx, cy) камеры глубины (по умолчанию — из констант).
    :param window: размер окна медианы при сэмплировании глубины (нечётный).
    :return: (X, Y, Z) или None, если глубина в цели невалидна/за кадром.
    """
    rgb_k = rgb_k or intrinsics(RGB_WIDTH, RGB_HEIGHT, RGB_HFOV)
    depth_k = depth_k or intrinsics(DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_HFOV)
    fx_r, fy_r, cx_r, cy_r = rgb_k
    fx_d, fy_d, cx_d, cy_d = depth_k

    u_r, v_r = bbox_center(bbox)
    tan_h = (u_r - cx_r) / fx_r   # >0 вправо по кадру -> +Y (влево в мире)
    tan_v = (cy_r - v_r) / fy_r   # >0 вверх по кадру -> +Z (вверх в мире)

    # Тот же луч в пикселях карты глубины (сенсоры совпадают, разные FOV).
    u_d = int(round(cx_d + fx_d * tan_h))
    v_d = int(round(cy_d - fy_d * tan_v))

    if depth.ndim != 2:
        raise ValueError('depth должен быть двумерным массивом (H, W)')
    h, w = depth.shape
    half = max(0, window // 2)
    y0, y1 = max(0, v_d - half), min(h, v_d + half + 1)
    x0, x1 = max(0, u_d - half), min(w, u_d + half + 1)
    if y1 <= y0 or x1 <= x0:
        return None

    roi = depth[y0:y1, x0:x1]
    roi = roi[np.isfinite(roi) & (roi > 0.0)]
    if roi.size == 0:
        return None

    z = float(np.median(roi))
    return (z, z * tan_h, z * tan_v)
