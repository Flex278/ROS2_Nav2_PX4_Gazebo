#!/usr/bin/env python3
"""Миссия из нескольких waypoints через Nav2 (/navigate_to_pose).

Последовательно отправляет цели в Nav2, дожидаясь SUCCEEDED каждой.
После последней цели (возврат на стартовую площадку) дрон выполняет
автопосадку через stop_timeout в nav2_commander_node.

Запуск:
    # терминал 1 — симуляция + Nav2 + nav2_commander_node:
    ros2 launch uav_bringup sim_full.launch.py mission_mode:=nav2

    # терминал 2 — миссия:
    python3 scripts/nav2_mission.py

Аргументы:
    --speed       скорость движения [м/с] (по умолчанию 1.5)
    --timeout     таймаут одной цели [с] (по умолчанию 120)
    --frame       frame_id цели (по умолчанию 'odom')
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

# ─── waypoints миссии (ENU: x вперёд, y влево) ─────────────────────────────
# Взлёт до 5 м уже выполняет nav2_commander_node, поэтому все точки на z=0
# (Nav2 планирует только в 2D, высоту держит commander).
MISSION_WAYPOINTS = [
    (0.0, 5.0, 0.0),    # 1. вправо 5 м
    (5.0, 5.0, 0.0),    # 2. вперёд 5 м
    (5.0, -5.0, 0.0),   # 3. влево 10 м
    (0.0, 0.0, 0.0),    # 4. назад → на оранжевый круг
]


class MultiGoalClient(Node):
    """Последовательная отправка нескольких целей в Nav2."""

    def __init__(self, speed: float = 1.5, frame: str = 'odom') -> None:
        super().__init__('nav2_mission_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._speed = speed
        self._frame = frame

        # Состояние текущей цели
        self._done = False
        self._succeeded = False
        self._goal_handle = None
        self._result_received = False

    def reset_goal_state(self) -> None:
        self._done = False
        self._succeeded = False
        self._goal_handle = None
        self._result_received = False

    def send_goal(self, x: float, y: float, yaw: float = 0.0,
                  timeout: float = 10.0) -> bool:
        """Отправить одну цель в /navigate_to_pose."""
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f'Action-сервер /navigate_to_pose недоступен ({timeout:.0f}с)')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'  → цель: x={x:+.2f}, y={y:+.2f}, yaw={yaw:.2f}')
        self._send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback)
        self._send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('  Goal отклонён сервером')
            self._done = True
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        p = fb.current_pose.pose.position
        self.get_logger().info(
            f'    fb: pos=({p.x:+.2f}, {p.y:+.2f}) '
            f'rem={fb.distance_remaining:.2f}м '
            f'rec={fb.number_of_recoveries}',
            throttle_duration_sec=2.0)

    def _on_result(self, future) -> None:
        self._result_received = True
        self._try_finalize()

    def _try_finalize(self) -> None:
        if self._done or not self._result_received:
            return
        status = (self._goal_handle.status
                  if self._goal_handle is not None
                  else GoalStatus.STATUS_UNKNOWN)
        if status in (GoalStatus.STATUS_SUCCEEDED,
                      GoalStatus.STATUS_ABORTED,
                      GoalStatus.STATUS_CANCELED):
            self._succeeded = status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(
                f'  ✓ результат: {status} '
                f'({"SUCCEEDED" if self._succeeded else "FAILED"})')
            self._done = True

    def cancel_current_goal(self) -> None:
        """Отменить активную цель Nav2 (иначе BT продолжает гнать дрона)."""
        if self._goal_handle is not None:
            self.get_logger().info('  Отмена активной цели Nav2...')
            cancel_future = self._goal_handle.cancel_goal_async()
            # Даём серверу время обработать отмену до выхода.
            try:
                cancel_future.result(timeout=5.0)
            except Exception:  # noqa: BLE001
                pass

    def spin_until_done(self, timeout_sec: float = 300.0) -> bool:
        """Крутить спин до завершения текущей цели."""
        deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=timeout_sec)
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._try_finalize()
            if self.get_clock().now() > deadline:
                self.get_logger().warn(
                    f'  Таймаут ({timeout_sec:.0f}с) — прерываем цель')
                self.cancel_current_goal()
                # Ждём подтверждения отмены, чтобы Nav2 остановил движение.
                cancel_deadline = self.get_clock().now() + \
                    rclpy.duration.Duration(seconds=5.0)
                while rclpy.ok() and not self._done and \
                        self.get_clock().now() < cancel_deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                break
        return self._succeeded

    def run_mission(self, waypoints: list[tuple[float, float, float]],
                    goal_timeout: float = 120.0) -> bool:
        """Выполнить всю миссию: последовательно все цели."""
        total = len(waypoints)
        self.get_logger().info(
            f'=== Миссия Nav2: {total} целей, скорость ~{self._speed} м/с ===')
        t_start = time.time()

        for i, (x, y, yaw) in enumerate(waypoints, 1):
            self.get_logger().info(
                f'--- Цель {i}/{total}: x={x:+.1f}, y={y:+.1f} ---')
            self.reset_goal_state()

            if not self.send_goal(x, y, yaw, timeout=10.0):
                self.get_logger().error(f'Не удалось отправить цель {i}')
                return False

            ok = self.spin_until_done(timeout_sec=goal_timeout)
            if not ok:
                self.get_logger().error(f'Цель {i} не достигнута (FAILED)')
                return False

            elapsed = time.time() - t_start
            self.get_logger().info(
                f'  ✓ цель {i}/{total} достигнута за {elapsed:.0f}с')

        elapsed = time.time() - t_start
        self.get_logger().info(
            f'=== Все {total} целей достигнуты за {elapsed:.0f}с ===')
        self.get_logger().info(
            'Дрон вернулся на стартовую площадку. '
            'Ожидайте автопосадку (stop_timeout → LAND → force-disarm).')
        return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Nav2 multi-waypoint mission')
    parser.add_argument('--speed', type=float, default=1.5,
                        help='Скорость движения [м/с]')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Таймаут одной цели [с]')
    parser.add_argument('--frame', default='odom',
                        help='frame_id цели (odom, т.к. map→odom нет)')
    parser.add_argument('--waypoints', nargs='+', type=float,
                        default=None,
                        help='Список waypoints: x1 y1 yaw1 x2 y2 yaw2 ... '
                             '(если не заданы — используется стандартный '
                             'маршрут миссии)')
    args = parser.parse_args()

    # Формируем список waypoints
    if args.waypoints:
        wp = args.waypoints
        if len(wp) % 3 != 0:
            print('Ошибка: waypoints должны быть тройками (x y yaw)')
            raise SystemExit(1)
        waypoints = [(wp[i], wp[i+1], wp[i+2])
                     for i in range(0, len(wp), 3)]
    else:
        waypoints = MISSION_WAYPOINTS

    print(f'Миссия: {len(waypoints)} целей, скорость {args.speed} м/с')
    for i, (x, y, yaw) in enumerate(waypoints, 1):
        print(f'  {i}. x={x:+.1f}  y={y:+.1f}  yaw={yaw:.1f}')

    rclpy.init()
    node = MultiGoalClient(speed=args.speed, frame=args.frame)
    code = 1
    try:
        ok = node.run_mission(waypoints, goal_timeout=args.timeout)
        code = 0 if ok else 1
    except KeyboardInterrupt:
        print('\nПрервано пользователем.')
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
