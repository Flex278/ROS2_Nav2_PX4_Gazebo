"""Локализация цели: /detections + /camera/depth -> /target/position (odom).

Берёт bbox из /detections (результат marker_detector / human_detector), берёт
последнюю карту глубины, восстанавливает 3D-точку в кадре камеры (X вперёд,
Y влево, Z вверх) через uav_perception.localization и трансформирует её в
`target_frame` (odom) через TF.

Публикует: /target/position (geometry_msgs/PointStamped, frame_id=target_frame).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener, TransformException

from uav_msgs.msg import TargetDetection
from uav_perception.cv_utils import image_to_numpy
from uav_perception.localization import (
    DEPTH_HEIGHT,
    DEPTH_HFOV,
    DEPTH_WIDTH,
    RGB_HEIGHT,
    RGB_HFOV,
    RGB_WIDTH,
    bbox_to_position,
    intrinsics,
)


class TargetLocalizerNode(Node):
    """Восстанавливает мировые координаты цели из bbox + глубины."""

    def __init__(self) -> None:
        super().__init__('target_localizer')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('depth_topic', '/camera/depth')
        self.declare_parameter('output_topic', '/target/position')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('max_depth_age', 1.0)  # сек симуляционного времени

        det_topic = self.get_parameter('detections_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._max_age_ns = int(float(self.get_parameter('max_depth_age').value) * 1e9)

        # Интринсики камер из model.sdf (без дисторсии, как в Gazebo).
        self._rgb_k = intrinsics(RGB_WIDTH, RGB_HEIGHT, RGB_HFOV)
        self._depth_k = intrinsics(DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_HFOV)

        self._depth = None  # (rclpy.time.Time, np.ndarray HxW float32 метры)

        self._sub_det = self.create_subscription(
            TargetDetection, det_topic, self._on_detection, 10)
        self._sub_depth = self.create_subscription(
            Image, depth_topic, self._on_depth, 10)
        self._pub = self.create_publisher(PointStamped, out_topic, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info('TargetLocalizerNode инициализирована')

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = image_to_numpy(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Ошибка декодирования глубины: {exc}', once=True)
            return
        self._depth = (self.get_clock().now(), depth)

    def _on_detection(self, msg: TargetDetection) -> None:
        if self._depth is None:
            return
        stamp, depth = self._depth
        if (self.get_clock().now() - stamp).nanoseconds > self._max_age_ns:
            return

        bbox = (msg.x_min, msg.y_min, msg.x_max, msg.y_max)
        pos = bbox_to_position(bbox, depth, self._rgb_k, self._depth_k)
        if pos is None:
            return

        # Кадр камеры берём из заголовка детекции (его detector скопировал из
        # /camera/image_raw). Резервный вариант — кадр сенсора RGB в x500_depth.
        camera_frame = msg.header.frame_id or 'x500_depth_0/OakD-Lite/base_link/IMX214'

        point_in = PointStamped()
        point_in.header.stamp = msg.header.stamp
        point_in.header.frame_id = camera_frame
        point_in.point.x = float(pos[0])
        point_in.point.y = float(pos[1])
        point_in.point.z = float(pos[2])

        try:
            point_out = self._tf_buffer.transform(point_in, self._target_frame)
        except TransformException as exc:  # noqa: BLE001
            self.get_logger().warn(f'Нет TF {camera_frame}->{self._target_frame}: {exc}',
                                  once=True)
            return

        self._pub.publish(point_out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
