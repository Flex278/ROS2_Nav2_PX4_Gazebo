#!/usr/bin/env python3
"""Публикация TF odom -> base_link и nav_msgs/Odometry из одометрии PX4.

Nav2 требует:
  1) TF odom -> base_link (costmap/planner/controller ждут эту трансформацию);
  2) nav_msgs/Odometry на /odom (bt_navigator и DWB берут оттуда скорость —
     без этого Nav2 не активируется и не выдаёт /cmd_vel).

PX4 отдаёт одометрию в /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry)
в NED. Здесь делаем плоскую 2D-проекцию в ROS-конвенции (ENU) и публикуем
TF и Odometry из одного источника, чтобы штампы времени совпадали.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster

from uav_sensors.odometry import (
    ned_to_enu_pose,
    ned_to_enu_twist,
    yaw_to_quaternion,
)


def px4_qos() -> QoSProfile:
    """QoS топиков px4_msgs: Best Effort + VOLATILE (как у offboard_node)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class OdomTFBroadcaster(Node):
    """Транслирует VehicleOdometry (NED) в TF odom->base_link и /odom (ENU).

    TF/Odometry публикуются по таймеру с постоянной частотой (а не только при
    приходе VehicleOdometry). При низком RTF симуляции PX4 шлёт одометрию
    рывками (паузы 0.3-0.6 с sim), из-за чего таймлайн TF имел разрывы, и
    MessageFilter costmap дропал свежие сканы как «earlier than all data».
    Постоянный таймер заполняет эти разрывы последним известным положением.
    """

    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self._br = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._on_odometry,
            px4_qos(),
        )
        # (enu_x, enu_y, enu_z, vx, vy, vz, wz, qz, qw) последней одометрии.
        self._latest = None
        # 50 Гц sim — гладкий таймлайн TF для tf2 (без разрывов).
        self._timer = self.create_timer(0.02, self._publish)
        self.get_logger().info('OdomTFBroadcaster инициализирована')

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        enu_x, enu_y, enu_z, yaw_enu = ned_to_enu_pose(msg.position, msg.q)
        vx, vy, vz, wz = ned_to_enu_twist(msg.velocity, msg.angular_velocity)
        qz, qw = yaw_to_quaternion(yaw_enu)
        self._latest = (enu_x, enu_y, enu_z, vx, vy, vz, wz, qz, qw)

    def _publish(self) -> None:
        if self._latest is None:
            return
        enu_x, enu_y, enu_z, vx, vy, vz, wz, qz, qw = self._latest
        stamp = self.get_clock().now().to_msg()

        # 1) TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = enu_x
        t.transform.translation.y = enu_y
        t.transform.translation.z = enu_z
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._br.sendTransform(t)

        # 2) nav_msgs/Odometry на /odom (нужна bt_navigator и DWB)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = wz

        # Диагональные ковариации (грубая оценка, точных данных PX4 не даёт).
        odom.pose.covariance[0] = 0.01    # x, ~10 см
        odom.pose.covariance[7] = 0.01    # y
        odom.pose.covariance[14] = 0.01   # z
        odom.pose.covariance[35] = 0.003  # yaw, ~3°
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[7] = 0.01   # vy
        odom.twist.covariance[14] = 0.01  # vz
        odom.twist.covariance[35] = 0.01  # wz
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
