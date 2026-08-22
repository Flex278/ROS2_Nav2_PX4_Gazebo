"""Тесты линейного MPC."""
import numpy as np

from _common import ROOT  # noqa: F401
from uav_offboard.mpc_controller import MPCController


def step_di(s, a, dt):
    p, v = s
    return np.array([p + v * dt + 0.5 * a * dt * dt, v + a * dt])


def test_track_constant_velocity():
    dt = 0.1
    mpc = MPCController(horizon=10, dt=dt, max_acc=5.0)
    s = np.zeros(6)
    for i in range(100):
        t = i * dt
        ref = np.column_stack([1.0 * (t + np.arange(10) * dt),
                               np.zeros(10), np.zeros(10)])
        u = mpc.compute_control(s, ref)
        assert np.all(np.abs(u) <= mpc.max_acc + 1e-9)
        for ax in range(3):
            s2 = step_di(np.array([s[ax], s[3 + ax]]), u[ax], dt)
            s[ax] = s2[0]
            s[3 + ax] = s2[1]
    # за ~10 c должен догнать цель (10 м)
    assert abs(s[0] - 10.0) < 0.5, f'x={s[0]:.3f}'
    assert abs(s[1]) < 1e-6 and abs(s[2]) < 1e-6


def test_hold_position_zero_control():
    mpc = MPCController()
    s = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
    ref = np.tile([1.0, 2.0, 3.0], (10, 1))
    u = mpc.compute_control(s, ref)
    assert np.allclose(u, 0.0, atol=1e-6)


def test_control_axis_decoupled():
    # коррекция только по оси x не должна трогать y/z
    mpc = MPCController()
    s = np.zeros(6)
    ref = np.column_stack([np.ones(10), np.zeros(10), np.zeros(10)])
    u = mpc.compute_control(s, ref)
    assert u[0] > 0.0
    assert abs(u[1]) < 1e-9 and abs(u[2]) < 1e-9


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
