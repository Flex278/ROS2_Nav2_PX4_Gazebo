"""Преобразование одометрии PX4 (NED/FRD) в ROS-конвенцию (ENU/FLU).

PX4 публикует /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry):
  position: [север, восток, вниз]  (NED)
  q: [w, x, y, z]  — поворот FRD -> NED
  velocity: [север, восток, вниз]  (NED)
  angular_velocity: телесные угловые скорости в FRD (z вниз)

ROS (Nav2) работает в ENU/FLU:
  x=восток, y=север, z=вверх, рысканье вокруг оси z вверх.
"""
from __future__ import annotations

import math


def ned_to_enu_pose(position, q) -> tuple[float, float, float, float]:
    """NED-позиция и кватернион FRD->NED -> (x, y, z, yaw) в ENU.

    :param position: [север, восток, вниз], м
    :param q: [w, x, y, z]
    :return: (x_enu, y_enu, z_enu, yaw_enu) в метрах и радианах
    """
    enu_x = float(position[1])   # восток
    enu_y = float(position[0])   # север
    enu_z = float(-position[2])  # вверх
    w, x, y, z = (float(c) for c in q)
    yaw_ned = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    yaw_enu = -yaw_ned  # разворот оси Z (NED -> ENU)
    return enu_x, enu_y, enu_z, yaw_enu


def ned_to_enu_twist(velocity, angular_velocity) -> tuple[float, float, float, float]:
    """Скорости NED/FRD -> (vx, vy, vz, wz) в ENU.

    :param velocity: [север, восток, вниз], м/с
    :param angular_velocity: телесные угловые скорости FRD [x, y, z], рад/с
    :return: (vx, vy, vz, wz) в ENU
    """
    vx = float(velocity[1])   # восток
    vy = float(velocity[0])   # север
    vz = float(-velocity[2])  # вверх
    wz = float(-angular_velocity[2])  # z вниз (FRD) -> z вверх (ENU)
    return vx, vy, vz, wz


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """Yaw (ENU, рад) -> (qz, qw) кватерниона вращения вокруг z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
