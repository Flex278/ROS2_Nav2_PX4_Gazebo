#!/usr/bin/env python3
"""Проверка состояния дрона: armed/nav_state/position."""
import sys
import time

import rclpy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("check_drone_state")
    state = {}

    def cb_status(m):
        state["armed"] = m.arming_state
        state["nav"] = m.nav_state

    def cb_pos(m):
        state["z"] = -m.z  # NED -> ENU
        state["x"] = m.x
        state["y"] = m.y

    node.create_subscription(
        VehicleStatus, "/fmu/out/vehicle_status", cb_status,
        qos_profile_sensor_data)
    node.create_subscription(
        VehicleLocalPosition, "/fmu/out/vehicle_local_position", cb_pos,
        qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < 6:
        rclpy.spin_once(node, timeout_sec=0.2)
        if "armed" in state and "z" in state:
            break

    print("arming_state=%s nav_state=%s pos=(%.2f, %.2f, z=%.2f ENU)"
          % (state.get("armed"), state.get("nav"),
             state.get("x", 0.0), state.get("y", 0.0), state.get("z", 0.0)))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
