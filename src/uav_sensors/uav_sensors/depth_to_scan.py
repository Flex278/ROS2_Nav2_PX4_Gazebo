"""Преобразование карты глубины в лазерный скан для Nav2.

Nav2 работает с 2D-сканом (/scan, sensor_msgs/LaserScan). Эта нода берёт
глубинную карту с камеры дрона (/camera/depth) и проецирует центральную
полосу пикселей в лазерный скан.

Публикует: /scan (sensor_msgs/LaserScan)
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from uav_sensors.scan_projection import depth_row_to_scan


class DepthToScanNode(Node):
    """Нода преобразования depth image -> LaserScan."""

    def __init__(self) -> None:
        super().__init__('depth_to_scan')
        self.declare_parameter('depth_topic', '/camera/depth')
        self.declare_parameter('scan_topic', '/scan')
        # Горизонтальный FOV камеры глубины OakD-Lite (StereoOV7251):
        # horizontal_fov=1.274 рад ≈ 73° (см. Tools/simulation/gz/models/OakD-Lite).
        self.declare_parameter('fov_h_deg', 73.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('frame_id', 'camera_depth_frame')

        depth = self.get_parameter('depth_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(Image, depth, self._on_depth, 10)
        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

        self._fov = float(self.get_parameter('fov_h_deg').value) * np.pi / 180.0
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._frame_id = self.get_parameter('frame_id').value
        self.get_logger().info('DepthToScanNode инициализирована')

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._decode_depth(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Ошибка декодирования глубины: {e}', once=True)
            return

        if depth.shape[0] == 0 or depth.shape[1] == 0:
            return
        center_row = depth[depth.shape[0] // 2, :]
        angles, ranges = depth_row_to_scan(
            center_row, self._fov, self._range_min, self._range_max)

        w = len(center_row)
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self._frame_id
        scan.angle_min = float(angles[0])
        scan.angle_max = float(angles[-1])
        scan.angle_increment = self._fov / (w - 1) if w > 1 else 0.0
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges.tolist()
        scan.intensities = []
        self._scan_pub.publish(scan)

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray:
        if msg.encoding == '32FC1':
            arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        elif msg.encoding == '16UC1':
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).astype(
                np.float32) / 1000.0
        else:
            raise ValueError(f'Неподдерживаемая кодировка глубины: {msg.encoding}')
        return arr.reshape(msg.height, msg.width)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
