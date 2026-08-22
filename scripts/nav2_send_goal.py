#!/usr/bin/env python3
"""Mission-level тест Nav2: отправка goal через /navigate_to_pose.

Поднимаем полную симуляцию в режиме Nav2 (mission_mode:=nav2), затем из
этого скрипта отправляем цель в action-сервер Nav2 (/navigate_to_pose).
nav2_commander_node подхватывает /cmd_vel, ведёт дрон к цели, а после
остановки и подтверждения касания земли делает force-disarm.

Запуск (в контейнере, с установленным оверлеем рабочего пространства):
    # терминал 1 — симуляция + Nav2 + nav2_commander_node:
    ros2 launch uav_bringup sim_full.launch.py mission_mode:=nav2

    # терминал 2 — цель:
    python3 scripts/nav2_send_goal.py --x 3.0 --y 0.0 --yaw 0.0

Аргументы:
    --x, --y, --yaw  целевая поза в odom-кадре (yaw в радианах, ENU)
    --frame          frame_id цели (по умолчанию 'odom': map->odom в стенде нет)
    --timeout        таймаут ожидания action-сервера [с] (по умолчанию 10)
"""
from __future__ import annotations

import argparse
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class NavigateToPoseClient(Node):
    """Клиент action-сервера Nav2 /navigate_to_pose."""

    def __init__(self) -> None:
        super().__init__('nav2_goal_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._done = False
        self._succeeded = False
        self._goal_handle = None
        self._result_received = False

    @property
    def done(self) -> bool:
        return self._done

    @property
    def succeeded(self) -> bool:
        return self._succeeded

    def send_goal(self, x: float, y: float, yaw: float,
                  frame: str = 'map', timeout: float = 10.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                'Action-сервер /navigate_to_pose недоступен '
                f'({timeout:.0f}с). Nav2 запущен?')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        # yaw -> кватернион (вращение только вокруг z).
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Отправка goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} '
            f'(frame={frame})')
        self._send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback)
        self._send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal отклонён сервером')
            self._done = True
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal принят, выполняю...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        p = fb.current_pose.pose.position
        self.get_logger().info(
            f'  feedback: pos=({p.x:+.2f}, {p.y:+.2f}) '
            f'remaining={fb.distance_remaining:.2f}м '
            f'recoveries={fb.number_of_recoveries}')

    def _on_result(self, future) -> None:
        # Result пришёл, но финальный статус goal_handle может обновиться чуть
        # позже (гонка result- и status-колбэков: result-колбэк иногда срабатывает
        # раньше, чем клиент обработает статус SUCCEEDED). Фиксируем факт прихода
        # результата, а итоговый статус читаем в spin_until_done().
        self._result_received = True
        self._try_finalize()

    def _try_finalize(self) -> None:
        """Завершает ожидание, когда пришёл result и статус стал терминальным."""
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
                f'Результат Nav2: status={status} '
                f'({"SUCCEEDED" if self._succeeded else "FAILED"})')
            self._done = True

    def spin_until_done(self, timeout_sec: float = 300.0) -> None:
        """Крутит спин до прихода результата и терминального статуса.

        Основной цикл обрабатывает и result-, и status-колбэки, поэтому
        здесь надёжно читается финальный статус goal_handle (без гонки).
        """
        deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=timeout_sec)
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._try_finalize()
            if self.get_clock().now() > deadline:
                self.get_logger().warn(
                    'Таймаут ожидания результата Nav2 — выходим без результата')
                break


def main() -> None:
    parser = argparse.ArgumentParser(description='Nav2 mission-level goal')
    parser.add_argument('--x', type=float, default=3.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--frame', default='odom',
                        help="frame_id цели; в этой конфигурации Nav2 "
                             "работает в odom (map->odom нет)")
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavigateToPoseClient()
    try:
        if node.send_goal(args.x, args.y, args.yaw, args.frame, args.timeout):
            node.spin_until_done()
        code = 0 if node.succeeded else 1
        if node.succeeded:
            print('ИТОГ: Nav2 достиг цели (SUCCEEDED). '
                  'Дрон должен выполнить автопосадку и force-disarm.')
        else:
            print('ИТОГ: Nav2 не достиг цели.')
    except KeyboardInterrupt:
        code = 1
        print('Прервано пользователем.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
