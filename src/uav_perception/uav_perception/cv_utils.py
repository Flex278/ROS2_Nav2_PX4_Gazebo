"""Утилиты конвертации ROS Image <-> numpy (без зависимости от cv_bridge).

Поддерживаемые кодировки: mono8/8UC1, bgr8, rgb8, 16UC1 (мм), 32FC1 (м).
"""
from __future__ import annotations

import numpy as np

_ENCODING_TO_DTYPE = {
    'mono8': (np.uint8, 1),
    '8UC1': (np.uint8, 1),
    '8UC3': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}


def image_to_numpy(msg) -> np.ndarray:
    """sensor_msgs/Image -> numpy (H, W[, C])."""
    dtype, channels = _ENCODING_TO_DTYPE[msg.encoding]
    data = np.frombuffer(bytes(msg.data), dtype=dtype)
    if channels == 1:
        return data.reshape(msg.height, msg.width)
    return data.reshape(msg.height, msg.width, channels)


def depth_to_meters(img: np.ndarray, encoding: str) -> np.ndarray:
    """Карта глубины -> метры (float32)."""
    if encoding == '32FC1':
        return img.astype(np.float32)
    if encoding == '16UC1':
        return img.astype(np.float32) / 1000.0
    raise ValueError(f'Неподдерживаемая кодировка глубины: {encoding}')


def to_rgb(img: np.ndarray, encoding: str) -> np.ndarray:
    """Привести изображение к RGB (для YOLO)."""
    if encoding in ('bgr8', '8UC3'):
        return img[:, :, ::-1]
    if img.ndim == 2:  # mono -> дублируем каналы
        return np.repeat(img[:, :, None], 3, axis=2)
    return img


def to_bgr(img: np.ndarray, encoding: str) -> np.ndarray:
    """Привести изображение к BGR (для OpenCV)."""
    if encoding in ('rgb8', '8UC3'):
        return img[:, :, ::-1]
    if img.ndim == 2:  # mono -> дублируем каналы
        return np.repeat(img[:, :, None], 3, axis=2)
    return img
