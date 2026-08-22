"""Тесты локализации bbox -> 3D (чистая логика, без rclpy)."""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_perception.localization import (
    DEPTH_HEIGHT,
    DEPTH_HFOV,
    DEPTH_WIDTH,
    RGB_HEIGHT,
    RGB_HFOV,
    RGB_WIDTH,
    bbox_to_position,
    intrinsics,
    vertical_fov,
)


def test_intrinsics_center():
    fx, fy, cx, cy = intrinsics(RGB_WIDTH, RGB_HEIGHT, RGB_HFOV)
    assert cx == RGB_WIDTH / 2.0
    assert cy == RGB_HEIGHT / 2.0
    assert fx > 0 and fy > 0


def test_vertical_fov_less_than_horizontal():
    v = vertical_fov(RGB_HFOV, RGB_WIDTH, RGB_HEIGHT)
    assert 0.0 < v < RGB_HFOV


def test_straight_ahead_center():
    depth = np.full((DEPTH_HEIGHT, DEPTH_WIDTH), 4.0, dtype=np.float32)
    bbox = (RGB_WIDTH // 2 - 2, RGB_HEIGHT // 2 - 2,
            RGB_WIDTH // 2 + 2, RGB_HEIGHT // 2 + 2)
    pos = bbox_to_position(bbox, depth)
    assert pos is not None
    assert np.isclose(pos[0], 4.0, atol=0.05)
    assert np.isclose(pos[1], 0.0, atol=0.05)
    assert np.isclose(pos[2], 0.0, atol=0.05)


def test_roundtrip_known_point():
    # Точка (5, 1, 0.5) в кадре камеры должна восстановиться из её проекции.
    fx_r, fy_r, cx_r, cy_r = intrinsics(RGB_WIDTH, RGB_HEIGHT, RGB_HFOV)
    X, Y, Z = 5.0, 1.0, 0.5
    u_r = cx_r + fx_r * (Y / X)
    v_r = cy_r - fy_r * (Z / X)
    depth = np.full((DEPTH_HEIGHT, DEPTH_WIDTH), X, dtype=np.float32)
    u, v = int(round(u_r)), int(round(v_r))
    bbox = (u - 3, v - 3, u + 3, v + 3)
    pos = bbox_to_position(bbox, depth)
    assert pos is not None
    assert np.isclose(pos[0], X, atol=0.1)
    assert np.isclose(pos[1], Y, atol=0.1)
    assert np.isclose(pos[2], Z, atol=0.1)


def test_above_target_positive_z():
    # Цель выше оптической оси (v < cy) -> Z > 0 (вверх).
    fx_r, fy_r, cx_r, cy_r = intrinsics(RGB_WIDTH, RGB_HEIGHT, RGB_HFOV)
    depth = np.full((DEPTH_HEIGHT, DEPTH_WIDTH), 3.0, dtype=np.float32)
    v = int(cy_r) - 30
    bbox = (int(cx_r) - 3, v - 3, int(cx_r) + 3, v + 3)
    pos = bbox_to_position(bbox, depth)
    assert pos is not None
    assert pos[2] > 0.0


def test_invalid_depth_returns_none():
    depth = np.zeros((DEPTH_HEIGHT, DEPTH_WIDTH), dtype=np.float32)
    bbox = (10, 10, 30, 30)
    assert bbox_to_position(bbox, depth) is None


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
