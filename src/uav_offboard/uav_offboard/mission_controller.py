"""Чистый автомат состояний миссии (без rclpy) — тестируется локально.

Последовательность: взлёт -> зависание -> полёт по точкам -> посадка -> disarm.
Все координаты ENU (z вверх). Перевод в NED выполняет offboard_node при публикации.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from uav_offboard.trajectory import Trajectory, constant_speed_trajectory

# Константы nav_state из px4_msgs/msg/VehicleStatus.msg
NAV_STATE_OFFBOARD = 14
NAV_STATE_AUTO_LAND = 18


class MissionState(Enum):
    INIT = auto()
    ARMING = auto()
    SETTING_OFFBOARD = auto()
    TAKEOFF = auto()
    HOVER = auto()
    WAYPOINTS = auto()
    LANDING = auto()
    DISARMING = auto()
    DONE = auto()


class Command(Enum):
    ARM = auto()
    DISARM = auto()
    SET_OFFBOARD = auto()
    LAND = auto()


@dataclass
class StepResult:
    """Результат одного шага автомата."""

    commands: list[Command] = field(default_factory=list)
    setpoint: np.ndarray | None = None   # ENU (x, y, z), z вверх
    yaw: float = 0.0


class MissionController:
    """Автомат миссии: взлёт -> зависание -> точки -> посадка -> disarm."""

    def __init__(
        self,
        takeoff_height: float = 2.0,
        hover_time: float = 3.0,
        waypoints: list[list[float]] | None = None,
        climb_speed: float = 0.5,
        waypoint_speed: float = 0.5,
        dt: float = 0.1,
        position_tol: float = 0.2,
        land_tol: float = 0.3,
        touch_tol: float = 0.12,
        touch_debounce: float = 0.3,
        land_timeout: float = 20.0,
    ) -> None:
        self.takeoff_height = takeoff_height
        self.hover_time = hover_time
        self.waypoints = [np.asarray(w, dtype=float) for w in (waypoints or [])]
        self.climb_speed = climb_speed
        self.waypoint_speed = waypoint_speed
        self.dt = dt
        self.position_tol = position_tol
        self.land_tol = land_tol
        self.touch_tol = touch_tol
        self.touch_debounce = touch_debounce
        self.land_timeout = land_timeout

        self.state = MissionState.INIT
        self._t0: float | None = None
        self._state_start: float = 0.0
        self._traj: Trajectory | None = None
        self._traj_t0: float = 0.0
        self._touch_start: float | None = None

    @property
    def finished(self) -> bool:
        return self.state is MissionState.DONE

    def step(
        self,
        armed: bool,
        nav_state: int,
        position_enu: np.ndarray,
        now: float,
        dist_bottom: float | None = None,
    ) -> StepResult:
        """Один шаг автомата.

        :param armed: флаг arming (из VehicleStatus.arming_state)
        :param nav_state: текущий nav_state (VehicleStatus.nav_state)
        :param position_enu: текущая позиция ENU (x, y, z вверх)
        :param now: текущее время [с]
        :param dist_bottom: дистанция до земли [м] (или None, если датчик невалиден)
        """
        if self._t0 is None:
            self._t0 = now

        res = StepResult()
        alt = float(position_enu[2])
        state = self.state

        if state is MissionState.INIT:
            # Пауза на стабилизацию после старта
            if now - self._t0 >= 1.0:
                self._transition(MissionState.ARMING, now)

        elif state is MissionState.ARMING:
            res.commands.append(Command.ARM)
            if armed:
                self._transition(MissionState.SETTING_OFFBOARD, now)

        elif state is MissionState.SETTING_OFFBOARD:
            res.commands.append(Command.SET_OFFBOARD)
            if nav_state == NAV_STATE_OFFBOARD:
                self._transition(MissionState.TAKEOFF, now)

        elif state is MissionState.TAKEOFF:
            elapsed = now - self._state_start
            target_alt = min(self.takeoff_height, self.climb_speed * elapsed)
            res.setpoint = np.array([position_enu[0], position_enu[1], target_alt])
            if alt >= self.takeoff_height - self.position_tol:
                self._transition(MissionState.HOVER, now)

        elif state is MissionState.HOVER:
            res.setpoint = np.array(
                [position_enu[0], position_enu[1], self.takeoff_height])
            if now - self._state_start >= self.hover_time:
                if self.waypoints:
                    self._start_waypoints(position_enu, now)
                else:
                    self._transition(MissionState.LANDING, now)

        elif state is MissionState.WAYPOINTS:
            sp = self._sample_trajectory(now)
            if sp is None:
                self._transition(MissionState.LANDING, now)
            else:
                res.setpoint = sp

        elif state is MissionState.LANDING:
            res.commands.append(Command.LAND)
            # Основной путь: точное подтверждение касания земли по дальномеру
            # (dist_bottom). Переход к disarm только после подтверждённого
            # касания — тогда force-disarm выключает моторы уже на земле.
            if dist_bottom is not None:
                if dist_bottom < self.touch_tol:
                    if self._touch_start is None:
                        self._touch_start = now
                    if now - self._touch_start >= self.touch_debounce:
                        self._transition(MissionState.DISARMING, now)
                else:
                    self._touch_start = None
                # Страховка: если долго «около земли», но датчик касание не
                # подтвердил (глюк/шум) — всё равно переходим к disarm.
                if dist_bottom < self.land_tol and \
                        now - self._state_start > self.land_timeout:
                    self._transition(MissionState.DISARMING, now)
            else:
                # Датчик недоступен — fallback по высоте EKF (z вверх).
                if alt < self.land_tol and now - self._state_start > 1.0:
                    self._transition(MissionState.DISARMING, now)

        elif state is MissionState.DISARMING:
            res.commands.append(Command.DISARM)
            if not armed:
                self._transition(MissionState.DONE, now)

        return res

    # --- вспомогательные ---
    def _transition(self, new_state: MissionState, now: float) -> None:
        self.state = new_state
        self._state_start = now

    def _start_waypoints(self, position_enu: np.ndarray, now: float) -> None:
        pts = [position_enu] + self.waypoints
        self._traj = constant_speed_trajectory(
            pts, self.waypoint_speed, self.dt, hover_time=1.0)
        self._traj_t0 = now
        self._transition(MissionState.WAYPOINTS, now)

    def _sample_trajectory(self, now: float) -> np.ndarray | None:
        t = now - self._traj_t0
        if self._traj is None or t > self._traj.times[-1]:
            return None
        return self._traj.sample(t).copy()
