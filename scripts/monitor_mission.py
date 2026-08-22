#!/usr/bin/env python3
"""Мониторинг ключевых величин во время Nav2-миссии.

Логирует каждые 2 с: позицию (ENU), высоту, скорость из /odom, cmd_vel от DWB,
ближние точки /scan и занятые клетки локальной costmap. Позволяет отличить
медленный DWB от медленного дрона (PX4) и увидеть просадку высоты/стену.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__('mission_monitor')
        self.z = 0.0
        self.vz = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.scan_near = 0
        self.scan_min = float('inf')
        self.cost_occ = 0

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_pos, qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 5)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 5)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_cost, 5)

    def _on_pos(self, m: VehicleLocalPosition) -> None:
        # NED -> ENU высота
        self.z = -float(m.z)
        self.vz = -float(m.vz)

    def _on_odom(self, m: Odometry) -> None:
        self.odom_vx = float(m.twist.twist.linear.x)
        self.odom_vy = float(m.twist.twist.linear.y)

    def _on_cmd(self, m: Twist) -> None:
        self.cmd_vx = float(m.linear.x)
        self.cmd_vy = float(m.linear.y)

    def _on_scan(self, m: LaserScan) -> None:
        n = 0
        mn = float('inf')
        for r in m.ranges:
            if math.isfinite(r) and r > 0.01:
                if r < mn:
                    mn = r
                if r < 3.0:
                    n += 1
        self.scan_min = mn
        self.scan_near = n

    def _on_cost(self, m: OccupancyGrid) -> None:
        self.cost_occ = sum(1 for v in m.data if v > 50)


def main() -> None:
    rclpy.init()
    node = Monitor()
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            t = time.time() - t0
            print(
                f't={t:6.1f}s  z={node.z:5.2f}  vz={node.vz:+5.2f}  '
                f'odom_v=({node.odom_vx:+5.2f},{node.odom_vy:+5.2f})  '
                f'cmd=({node.cmd_vx:+5.2f},{node.cmd_vy:+5.2f})  '
                f'scan_min={node.scan_min:5.2f} scan<3м={node.scan_near:3d}  '
                f'cost_occ={node.cost_occ:3d}',
                flush=True)
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
