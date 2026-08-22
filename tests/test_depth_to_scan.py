"""Тесты проекции карты глубины в LaserScan."""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_sensors.scan_projection import depth_row_to_scan


def test_angles_and_ranges():
    row = np.array([1.0, 2.0, 3.0, 4.0, 5.0], dtype=np.float32)
    angles, ranges = depth_row_to_scan(
        row, fov_h=np.pi / 2, range_min=0.1, range_max=10.0)
    assert len(angles) == 5
    assert np.isclose(angles[0], -np.pi / 4)
    assert np.isclose(angles[-1], np.pi / 4)
    assert np.isclose(ranges[2], 3.0)  # центральный луч (angle=0)


def test_invalid_depth_is_inf():
    row = np.array([0.0, np.nan, 2.0])
    _, ranges = depth_row_to_scan(
        row, fov_h=1.0, range_min=0.1, range_max=10.0)
    assert np.isinf(ranges[0])
    assert np.isinf(ranges[1])
    assert np.isfinite(ranges[2])


def test_range_clipping():
    # 3 луча с углами [-0.5, 0, 0.5]: дальность = depth / cos(angle)
    row = np.array([0.05, 100.0, 1.0])
    _, ranges = depth_row_to_scan(
        row, fov_h=1.0, range_min=0.1, range_max=10.0)
    assert np.isclose(ranges[0], 0.1)
    assert np.isclose(ranges[1], 10.0)
    assert np.isclose(ranges[2], 1.0 / np.cos(0.5))


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
