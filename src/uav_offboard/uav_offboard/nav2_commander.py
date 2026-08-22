"""Чистый автомат для режима следования за Nav2 (без rclpy) — тестируется локально.

Жизненный цикл: взлёт -> следование за /cmd_vel (Twist -> position setpoint)
-> посадка -> force-disarm. Все координаты ENU (z вверх), перевод в NED
выполняет nav2_commander_node при публикации в PX4.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

# Константы nav_state из px4_msgs/msg/VehicleStatus.msg
NAV_STATE_OFFBOARD = 14
NAV_STATE_AUTO_LAND = 18


class Nav2State(Enum):
    INIT = auto()
    ARMING = auto()
    SETTING_OFFBOARD = auto()
    TAKEOFF = auto()
    FOLLOW = auto()
    LANDING = auto()
    DISARMING = auto()
    DONE = auto()


class Nav2Command(Enum):
    ARM = auto()
    DISARM = auto()
    SET_OFFBOARD = auto()
    LAND = auto()


@dataclass
class Nav2StepResult:
    """Результат одного шага автомата."""

    commands: list[Nav2Command] = field(default_factory=list)
    setpoint: np.ndarray | None = None   # ENU (x, y, z), z вверх
    yaw: float = 0.0


class Nav2Commander:
    """Автомат: взлёт -> следование за cmd_vel -> посадка -> force-disarm.

    Высота удерживается на ``takeoff_height``; горизонтальное движение
    задаётся «морковкой» (lookahead): целевая позиция = текущая + ``cmd_vel``
    * ``lookahead_time`` (расхождение ограничено ``max_ahead``).
    """

    def __init__(
        self,
        takeoff_height: float = 5.0,
        climb_speed: float = 0.5,
        dt: float = 0.1,
        position_tol: float = 0.2,
        max_speed: float = 1.5,
        stop_timeout: float = 15.0,
        touch_tol: float = 0.12,
        touch_debounce: float = 0.3,
        land_tol: float = 0.3,
        land_timeout: float = 10.0,
        max_ahead: float = 2.0,
        lookahead_time: float = 1.0,
    ) -> None:
        self.takeoff_height = takeoff_height
        self.climb_speed = climb_speed
        self.dt = dt
        self.position_tol = position_tol
        self.max_speed = max_speed
        self.stop_timeout = stop_timeout
        self.touch_tol = touch_tol
        self.touch_debounce = touch_debounce
        self.land_tol = land_tol
        self.land_timeout = land_timeout
        self.max_ahead = max_ahead
        self.lookahead_time = lookahead_time

        self.state = Nav2State.INIT
        self._t0: float | None = None
        self._state_start: float = 0.0
        self._touch_start: float | None = None
        self._target_pos: np.ndarray | None = None

        # Текущая команда скорости из Nav2 (ENU: x вперёд, y влево, z — yaw rate).
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        # Интегратор yaw (ENU, рад).
        self._yaw = 0.0
        # Признак «уже летели»: разрешает автопосадку по остановке cmd_vel.
        self._ever_moved = False
        self._stopped_since: float | None = None
        # Высота на момент входа в LANDING [м] — для активного снижения.
        self._landing_start_alt: float = 0.0

    @property
    def finished(self) -> bool:
        return self.state is Nav2State.DONE

    def set_velocity_cmd(self, vx: float, vy: float, wz: float) -> None:
        """Принять команду скорости из Nav2 (linear.x, linear.y, angular.z)."""
        self._vx = float(vx)
        self._vy = float(vy)
        self._wz = float(wz)

    def step(
        self,
        armed: bool,
        nav_state: int,
        position_enu: np.ndarray,
        now: float,
        dist_bottom: float | None = None,
    ) -> Nav2StepResult:
        """Один шаг автомата.

        :param armed: флаг arming (VehicleStatus.arming_state)
        :param nav_state: текущий nav_state (VehicleStatus.nav_state)
        :param position_enu: текущая позиция ENU (x, y, z вверх)
        :param now: текущее время [с]
        :param dist_bottom: дистанция до земли [м] (None — датчик невалиден)
        """
        if self._t0 is None:
            self._t0 = now

        res = Nav2StepResult()
        alt = float(position_enu[2])
        state = self.state

        if state is Nav2State.INIT:
            if now - self._t0 >= 1.0:
                self._transition(Nav2State.ARMING, now)

        elif state is Nav2State.ARMING:
            res.commands.append(Nav2Command.ARM)
            if armed:
                self._transition(Nav2State.SETTING_OFFBOARD, now)

        elif state is Nav2State.SETTING_OFFBOARD:
            res.commands.append(Nav2Command.SET_OFFBOARD)
            if nav_state == NAV_STATE_OFFBOARD:
                self._transition(Nav2State.TAKEOFF, now)
            elif not armed:
                # PX4 мог снять arm по переходной preflight-ошибке (например,
                # «Yaw estimate error», пока EKF2 не выровнял курс). Возвращаемся
                # к ARMING и пере-армим, когда EKF2 стабилизируется.
                self._transition(Nav2State.ARMING, now)

        elif state is Nav2State.TAKEOFF:
            if not armed:
                # Аналогично SETTING_OFFBOARD: preflight снял arm во время
                # набора высоты — возвращаемся к ARMING (пере-взлёт).
                self._transition(Nav2State.ARMING, now)
                return res
            elapsed = now - self._state_start
            target_alt = min(self.takeoff_height, self.climb_speed * elapsed)
            res.setpoint = np.array([position_enu[0], position_enu[1], target_alt])
            if alt >= self.takeoff_height - self.position_tol:
                self._transition(Nav2State.FOLLOW, now)

        elif state is Nav2State.FOLLOW:
            vx = float(np.clip(self._vx, -self.max_speed, self.max_speed))
            vy = float(np.clip(self._vy, -self.max_speed, self.max_speed))
            # Голономный дрон не поворачивается для движения (DWB имеет
            # max_vel_theta=0, критики поворота отключены), поэтому держим yaw
            # фиксированным — иначе дрон «крутится» вокруг вертикальной оси.
            self._yaw = 0.0

            # «Морковка на удочке» (lookahead): цель = текущая позиция + v * lookahead_time.
            # НЕ интегрируем cmd_vel: интеграция накапливает отставание и при
            # overshoot разворачивает цель ЗА дрон (цель прыгает ~2*max_ahead),
            # позиционный контроллер резко реверсирует и дрон «крутится/переворачивается».
            # При v=0 цель совпадает с позицией — overshoot при торможении не возникает.
            ahead = np.array([vx, vy], dtype=float) * self.lookahead_time
            dist = float(np.linalg.norm(ahead))
            if dist > self.max_ahead:
                ahead = ahead * (self.max_ahead / dist)

            self._target_pos = np.array(
                [position_enu[0] + ahead[0],
                 position_enu[1] + ahead[1],
                 self.takeoff_height + 0.5], dtype=float)

            res.setpoint = self._target_pos.copy()
            res.yaw = self._yaw

            speed = float(np.hypot(vx, vy))
            if speed > 0.05:
                self._ever_moved = True
                self._stopped_since = None
            elif self._ever_moved:
                if self._stopped_since is None:
                    self._stopped_since = now
                if now - self._stopped_since >= self.stop_timeout:
                    self._transition(Nav2State.LANDING, now)

        elif state is Nav2State.LANDING:
            res.commands.append(Nav2Command.LAND)
            # Активное снижение: setpoint опускается от высоты входа в LANDING
            # до 0.1 м со скоростью climb_speed. PX4 auto-land всё ещё активен
            # (команда LAND), но setpoint форсирует более быстрое снижение.
            elapsed = now - self._state_start
            target_z = max(0.1, self._landing_start_alt - self.climb_speed * elapsed)
            res.setpoint = np.array(
                [position_enu[0], position_enu[1], target_z], dtype=float)
            if dist_bottom is not None:
                if dist_bottom < self.touch_tol:
                    if self._touch_start is None:
                        self._touch_start = now
                    if now - self._touch_start >= self.touch_debounce:
                        self._transition(Nav2State.DISARMING, now)
                else:
                    self._touch_start = None
                if dist_bottom < self.land_tol and \
                        now - self._state_start > self.land_timeout:
                    self._transition(Nav2State.DISARMING, now)
            else:
                if alt < self.land_tol and now - self._state_start > 1.0:
                    self._transition(Nav2State.DISARMING, now)

        elif state is Nav2State.DISARMING:
            res.commands.append(Nav2Command.DISARM)
            if not armed:
                self._transition(Nav2State.DONE, now)

        return res

    def _transition(self, new_state: Nav2State, now: float) -> None:
        self.state = new_state
        self._state_start = now
        if new_state is Nav2State.TAKEOFF:
            # При новом взлёте сбрасываем интегрированную цель (иначе FOLLOW
            # продолжил бы со старой позиции).
            self._target_pos = None
        elif new_state is Nav2State.LANDING:
            # Запоминаем высоту входа в посадку для активного снижения.
            self._landing_start_alt = self._target_pos[2] if self._target_pos is not None else 0.0

