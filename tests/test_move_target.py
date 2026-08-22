"""Тесты чистой геометрии траекторий движущейся цели (scripts/move_target.py).

Проверяем только чистые функции circle_xy / pingpong_xy — без ROS и Gazebo.
"""
import os
import sys

from _common import ROOT  # noqa: F401

sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from move_target import circle_xy, pingpong_xy  # noqa: E402


def test_circle_starts_at_angle_zero():
    x, y = circle_xy(0.0, radius=3.0, cx=0.0, cy=0.0, period=20.0)
    assert abs(x - 3.0) < 1e-9
    assert abs(y - 0.0) < 1e-9


def test_circle_quarter_period():
    # t=5 с из периода 20 с -> четверть окружности -> угол 90° -> (0, +3).
    x, y = circle_xy(5.0, radius=3.0, cx=0.0, cy=0.0, period=20.0)
    assert abs(x) < 1e-9
    assert abs(y - 3.0) < 1e-9


def test_pingpong_endpoints():
    p0 = (10.0, 4.5)
    p1 = (10.0, -4.5)
    x, y = pingpong_xy(0.0, p0, p1, period=20.0)
    assert (abs(x - p0[0]) < 1e-9) and (abs(y - p0[1]) < 1e-9)
    # половина периода -> точка B
    x, y = pingpong_xy(10.0, p0, p1, period=20.0)
    assert abs(x - p1[0]) < 1e-9 and abs(y - p1[1]) < 1e-9
    # полный период -> снова точка A
    x, y = pingpong_xy(20.0, p0, p1, period=20.0)
    assert abs(x - p0[0]) < 1e-9 and abs(y - p0[1]) < 1e-9


def test_pingpong_midpoint():
    x, y = pingpong_xy(5.0, (0.0, 0.0), (4.0, 0.0), period=20.0)
    assert abs(x - 2.0) < 1e-9
    assert abs(y) < 1e-9


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')