"""Тесты преобразования одометрии PX4 (NED/FRD) -> ROS (ENU/FLU)."""
import math

from _common import ROOT  # noqa: F401
from uav_sensors.odometry import (
    ned_to_enu_pose,
    ned_to_enu_twist,
    yaw_to_quaternion,
)


def test_position_ned_to_enu():
    # NED [север=1, восток=2, вниз=-3] -> ENU [восток=2, север=1, вверх=3]
    x, y, z, _ = ned_to_enu_pose([1.0, 2.0, -3.0], [1.0, 0.0, 0.0, 0.0])
    assert (x, y, z) == (2.0, 1.0, 3.0)


def test_yaw_ned_to_enu_and_quaternion_roundtrip():
    # yaw_ned = +90° -> кватернион [w,0,0,z], yaw_enu должен быть -90°.
    w = math.cos(math.pi / 4)
    z = math.sin(math.pi / 4)
    _, _, _, yaw_enu = ned_to_enu_pose([0.0, 0.0, 0.0], [w, 0.0, 0.0, z])
    assert abs(yaw_enu - (-math.pi / 2)) < 1e-9

    # roundtrip yaw -> quaternion -> согласованные (qz, qw)
    qz, qw = yaw_to_quaternion(yaw_enu)
    assert abs(qz - math.sin((-math.pi / 2) / 2)) < 1e-9
    assert abs(qw - math.cos((-math.pi / 2) / 2)) < 1e-9


def test_twist_ned_to_enu():
    # velocity NED [север=1, восток=2, вниз=-3] -> [2,1,3]
    # angular z FRD +0.5 -> ENU -0.5
    vx, vy, vz, wz = ned_to_enu_twist([1.0, 2.0, -3.0], [0.0, 0.0, 0.5])
    assert (vx, vy, vz) == (2.0, 1.0, 3.0)
    assert wz == -0.5


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
