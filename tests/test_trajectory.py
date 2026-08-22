"""Тесты генерации траекторий."""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_offboard.trajectory import bezier_curve, constant_speed_trajectory


def test_bezier_endpoints():
    pts = [np.zeros(3), np.array([1.0, 1.0, 1.0]), np.array([2.0, 0.0, 0.0])]
    c = bezier_curve(pts, num_samples=50)
    assert np.allclose(c[0], pts[0], atol=1e-9)
    assert np.allclose(c[-1], pts[-1], atol=1e-9)


def test_constant_speed_trajectory():
    traj = constant_speed_trajectory(
        [np.zeros(3), np.array([1.0, 0.0, 0.0])],
        speed=0.5, dt=0.1, hover_time=1.0)
    assert np.all(np.diff(traj.times) > 0)
    assert traj.positions.shape == (len(traj.times), 3)
    assert traj.velocities.shape == (len(traj.times), 3)
    assert np.allclose(traj.positions[-1], [1.0, 0.0, 0.0])
    # x монотонно растёт, y/z ~ 0
    assert np.all(np.diff(traj.positions[:, 0]) >= -1e-9)
    assert np.allclose(traj.positions[:, 1], 0.0)
    assert np.allclose(traj.positions[:, 2], 0.0)


def test_sample_clamps():
    traj = constant_speed_trajectory([np.zeros(3)], speed=0.5, dt=0.1)
    assert np.allclose(traj.sample(999.0), traj.positions[-1])


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
