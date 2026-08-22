"""Нода режима Nav2: конвертирует /cmd_vel (geometry_msgs/Twist) в setpoint PX4.

Жизненный цикл (см. Nav2Commander): взлёт -> следование за /cmd_vel -> посадка
(с force-disarm после подтверждения касания земли по dist_bottom).

Запускается вместо offboard_node (миссия по точкам) — режим выбирается
аргументом ``mission_mode`` в sim_full.launch.py.
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from uav_offboard.nav2_commander import Nav2Command, Nav2Commander, Nav2State

# Константы VehicleCommand (px4_msgs)
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_NAV_LAND = 21
ARMING_ACTION_ARM = 1
ARMING_ACTION_DISARM = 0
ARM_DISARM_FORCE = 21196  # magic-число PX4: принудительный disarm (пропуск проверки «landed»)
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def px4_qos() -> QoSProfile:
    """QoS-профиль топиков px4_msgs (Best Effort + VOLATILE)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Nav2CommanderNode(Node):
    """Нода следования за Nav2: /cmd_vel -> TrajectorySetpoint."""

    def __init__(self) -> None:
        super().__init__('nav2_commander_node')
        qos = px4_qos()

        # --- Параметры ---
        self.declare_parameter('takeoff_height', 5.0)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('max_ahead', 1.5)
        self.declare_parameter('lookahead_time', 1.0)
        self.declare_parameter('stop_timeout', 15.0)

        # --- Публикаторы -> PX4 ---
        self._mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # --- Подписки <- PX4 ---
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_position, qos)

        # --- Подписка <- Nav2 ---
        self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # --- Автомат ---
        self._commander = Nav2Commander(
            takeoff_height=float(self.get_parameter('takeoff_height').value),
            max_speed=float(self.get_parameter('max_speed').value),
            max_ahead=float(self.get_parameter('max_ahead').value),
            lookahead_time=float(self.get_parameter('lookahead_time').value),
            stop_timeout=float(self.get_parameter('stop_timeout').value),
            dt=1.0 / float(self.get_parameter('update_rate').value),
        )

        # --- Текущее состояние ---
        self._position_enu = np.zeros(3)
        self._armed = False
        self._nav_state = 0
        self._dist_bottom: float | None = None
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        # --- Таймер ---
        period = 1.0 / float(self.get_parameter('update_rate').value)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info('Nav2CommanderNode инициализирована')

    # --- Обработчики подписок ---
    def _on_status(self, msg: VehicleStatus) -> None:
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._nav_state = msg.nav_state

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self._position_enu = np.array([msg.x, msg.y, -msg.z])
        self._dist_bottom = float(msg.dist_bottom) if msg.dist_bottom_valid else None

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Nav2 (DWB) выдаёт скорость в odom-кадре ENU: x=восток, y=север.
        # Внутри автомата конвенция [север, восток, вверх] (см. _on_position),
        # поэтому линейные компоненты меняем местами, иначе дрон едет
        # перпендикулярно цели (на север вместо востока).
        self._vx = msg.linear.y   # север  -> внутр. x (north)
        self._vy = msg.linear.x   # восток -> внутр. y (east)
        self._wz = msg.angular.z


    # --- Главный цикл ---
    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._commander.set_velocity_cmd(self._vx, self._vy, self._wz)
        res = self._commander.step(
            self._armed, self._nav_state, self._position_enu, now,
            self._dist_bottom)

        # Во время посадки/disarm НЕ публикуем OFFBOARD-режим и setpoint:
        # иначе PX4 не завершает AUTO_LAND (см. offboard_node.py).
        state = self._commander.state
        if state not in (Nav2State.LANDING, Nav2State.DISARMING, Nav2State.DONE):
            self._publish_control_mode()
            self._publish_setpoint(res)

        for cmd in res.commands:
            self._publish_command(cmd)

        if self._commander.finished:
            self.get_logger().info('Миссия Nav2 завершена')
            self.destroy_timer(self._timer)

    def _publish_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._mode_pub.publish(msg)

    def _publish_setpoint(self, res) -> None:
        enu = np.asarray(res.setpoint, dtype=float) if res.setpoint is not None \
            else self._position_enu
        ned = np.array([enu[0], enu[1], -enu[2]], dtype=np.float32)

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = ned.tolist()
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float(res.yaw)
        msg.yawspeed = float('nan')
        self._sp_pub.publish(msg)

    def _publish_command(self, cmd: Nav2Command) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        if cmd is Nav2Command.ARM:
            msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = float(ARMING_ACTION_ARM)
        elif cmd is Nav2Command.DISARM:
            msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = float(ARMING_ACTION_DISARM)
            msg.param2 = float(ARM_DISARM_FORCE)  # force-disarm
        elif cmd is Nav2Command.SET_OFFBOARD:
            msg.command = VEHICLE_CMD_DO_SET_MODE
            msg.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            msg.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        elif cmd is Nav2Command.LAND:
            msg.command = VEHICLE_CMD_NAV_LAND
        self._cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
