"""Offboard-нода управления квадрокоптером (PX4 SITL).

Публикует в PX4 (мост MicroXRCE-DDS, без MAVROS):
  /fmu/in/offboard_control_mode  (px4_msgs/OffboardControlMode)
  /fmu/in/trajectory_setpoint    (px4_msgs/TrajectorySetpoint)
  /fmu/in/vehicle_command        (px4_msgs/VehicleCommand)

Подписывается на:
  /fmu/out/vehicle_status        (px4_msgs/VehicleStatus)
  /fmu/out/vehicle_local_position(px4_msgs/VehicleLocalPosition)

ВАЖНО:
  1) QoS Best Effort + VOLATILE (иначе PX4 «не слышит» команды).
  2) PX4 работает в NED (z вниз). Внутри ноды — ENU (z вверх),
     перевод z -> -z выполняется при публикации setpoint.
  3) Setpoint нужно слать непрерывно: пауза > 0.5 c вызовет failsafe.
"""
from __future__ import annotations

import numpy as np
import rclpy
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

from uav_offboard.mission_controller import Command, MissionController, MissionState

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


class OffboardNode(Node):
    """Нода оффборд-управления."""

    def __init__(self) -> None:
        super().__init__('offboard_node')
        qos = px4_qos()

        # --- Параметры ---
        self.declare_parameter('takeoff_height', 2.0)
        self.declare_parameter('hover_time', 3.0)
        self.declare_parameter('update_rate', 30.0)
        # ROS2-параметры не поддерживают вложенные списки, поэтому маршрут
        # задаётся плоским списком [x0,y0,z0, x1,y1,z1, ...] (ENU, z вверх).
        self.declare_parameter(
            'waypoints',
            [0.0, 0.0, 2.0, 2.0, 0.0, 2.0, 2.0, 2.0, 2.0,
             0.0, 2.0, 2.0, 0.0, 0.0, 2.0],
        )

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

        # --- Автомат миссии ---
        wp_flat = self.get_parameter('waypoints').value
        waypoints = [list(wp_flat[i:i + 3]) for i in range(0, len(wp_flat), 3)]
        self._mission = MissionController(
            takeoff_height=float(self.get_parameter('takeoff_height').value),
            hover_time=float(self.get_parameter('hover_time').value),
            waypoints=waypoints,
            dt=1.0 / float(self.get_parameter('update_rate').value),
        )

        # --- Текущее состояние ---
        self._position_enu = np.zeros(3)
        self._armed = False
        self._nav_state = 0
        self._dist_bottom: float | None = None

        # --- Таймер ---
        period = 1.0 / float(self.get_parameter('update_rate').value)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info('OffboardNode инициализирована')

    # --- Обработчики подписок ---
    def _on_status(self, msg: VehicleStatus) -> None:
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._nav_state = msg.nav_state

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        # NED -> ENU (z вниз -> вверх)
        self._position_enu = np.array([msg.x, msg.y, -msg.z])
        # Дистанция до земли (валидна при наличии датчика расстояния/дальномера).
        self._dist_bottom = float(msg.dist_bottom) if msg.dist_bottom_valid else None

    # --- Главный цикл ---
    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        res = self._mission.step(
            self._armed, self._nav_state, self._position_enu, now,
            self._dist_bottom)

        # Во время посадки/disarm НЕ публикуем OFFBOARD-режим и setpoint:
        # иначе PX4 не может корректно завершить AUTO_LAND (land detector не
        # срабатывает -> «Disarming denied: not landed», дрон зависает).
        state = self._mission.state
        if state not in (MissionState.LANDING, MissionState.DISARMING,
                         MissionState.DONE):
            self._publish_control_mode()
            self._publish_setpoint(res)

        for cmd in res.commands:
            self._publish_command(cmd)

        if self._mission.finished:
            self.get_logger().info('Миссия завершена')
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
        # Если автомат не дал setpoint — держим текущую позицию
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

    def _publish_command(self, cmd: Command) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        if cmd is Command.ARM:
            msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = float(ARMING_ACTION_ARM)
        elif cmd is Command.DISARM:
            msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = float(ARMING_ACTION_DISARM)
            # Force-disarm: касание земли уже подтверждено по dist_bottom,
            # поэтому выключаем моторы без ожидания land detector PX4
            # (иначе «Disarming denied: not landed» в течение десятков секунд).
            msg.param2 = float(ARM_DISARM_FORCE)
        elif cmd is Command.SET_OFFBOARD:
            msg.command = VEHICLE_CMD_DO_SET_MODE
            msg.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            msg.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        elif cmd is Command.LAND:
            msg.command = VEHICLE_CMD_NAV_LAND
        self._cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
