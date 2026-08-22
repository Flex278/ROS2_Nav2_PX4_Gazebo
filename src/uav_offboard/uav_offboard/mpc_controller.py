"""Линейный MPC для слежения за траекторией (декомпозиция по осям).

Модель на ось — двойной интегратор:
    p[k+1] = p[k] + v[k]*dt + 0.5*a[k]*dt^2
    v[k+1] = v[k] + a[k]*dt

Задача решается как безусловная QP в сжатой форме (матрицы H, g) через numpy.
Управление ограничивается по модулю (max_acc) после решения.
"""
from __future__ import annotations

import numpy as np


class MPCController:
    """Линейный MPC (двойной интегратор, 3 независимые оси)."""

    def __init__(
        self,
        horizon: int = 10,
        dt: float = 0.1,
        q_pos: float = 10.0,
        q_vel: float = 1.0,
        r_acc: float = 0.1,
        max_acc: float = 2.0,
    ) -> None:
        self.horizon = horizon
        self.dt = dt
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_acc = r_acc
        self.max_acc = max_acc
        self._A, self._B = self._build_matrices(horizon, dt)

    @staticmethod
    def _build_matrices(horizon: int, dt: float):
        """Построить матрицы предсказания A (2N x 2) и B (2N x N)."""
        A2 = np.array([[1.0, dt], [0.0, 1.0]])
        B2 = np.array([[0.5 * dt * dt], [dt]])
        A = np.zeros((2 * horizon, 2))
        B = np.zeros((2 * horizon, horizon))
        for k in range(1, horizon + 1):
            row = slice(2 * (k - 1), 2 * k)
            A[row, :] = np.linalg.matrix_power(A2, k)
            for j in range(k):
                B[row, j] = (np.linalg.matrix_power(A2, k - 1 - j) @ B2).ravel()
        return A, B

    def compute_control(self, state, reference) -> np.ndarray:
        """Вычислить управление [ax, ay, az].

        :param state: (6,) = [x, y, z, vx, vy, vz]
        :param reference: (N, 3) опорные позиции [x, y, z]
        :return: (3,) ускорения [ax, ay, az]
        """
        state = np.asarray(state, dtype=float).reshape(6)
        reference = np.asarray(reference, dtype=float)
        if reference.ndim == 1:
            reference = reference.reshape(1, 3)
        ref = self._expand_reference(reference)

        u = np.zeros(3)
        for axis in range(3):
            s0 = np.array([state[axis], state[3 + axis]])
            u[axis] = self._solve_axis(s0, ref[:, axis])
        return np.clip(u, -self.max_acc, self.max_acc)

    def _expand_reference(self, reference: np.ndarray) -> np.ndarray:
        """Довести опорную траекторию до длины горизонта (повтор последней точки)."""
        n = reference.shape[0]
        if n >= self.horizon:
            return reference[: self.horizon]
        rep = np.repeat(reference[-1:], self.horizon - n, axis=0)
        return np.vstack([reference, rep])

    def _solve_axis(self, s0: np.ndarray, p_ref: np.ndarray) -> float:
        """Решить QP для одной оси и вернуть первое управление."""
        N = self.horizon
        v_ref = np.gradient(p_ref, self.dt)
        s_ref = np.empty(2 * N)
        s_ref[0::2] = p_ref
        s_ref[1::2] = v_ref

        Q = np.diag([self.q_pos, self.q_vel])
        Qbar = np.kron(np.eye(N), Q)
        Rbar = np.eye(N) * self.r_acc

        H = 2.0 * (self._B.T @ Qbar @ self._B + Rbar)
        g = 2.0 * self._B.T @ Qbar @ (self._A @ s0 - s_ref)
        u_vec = np.linalg.solve(H, -g)
        return float(u_vec[0])
