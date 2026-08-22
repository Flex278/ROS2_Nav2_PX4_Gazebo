"""Детектор H-маркера (OpenCV) для точной посадки.

Подписывается на /camera/image_raw (sensor_msgs/Image),
публикует /detections (uav_msgs/TargetDetection).

OpenCV живёт в отдельном venv (см. scripts/setup_venv.sh).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from uav_msgs.msg import TargetDetection
from uav_perception.cv_utils import image_to_numpy, to_bgr
from uav_perception.detection import detect_h_marker


class MarkerDetectorNode(Node):
    """Нода детекции H-маркера."""

    def __init__(self) -> None:
        super().__init__('marker_detector')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')

        camera = self.get_parameter('camera_topic').value
        det_topic = self.get_parameter('detection_topic').value
        self.create_subscription(Image, camera, self._on_image, 10)
        self._pub = self.create_publisher(TargetDetection, det_topic, 10)
        self.get_logger().info('MarkerDetectorNode инициализирована')

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = to_bgr(image_to_numpy(msg), msg.encoding)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Ошибка декодирования кадра: {e}', once=True)
            return

        det = detect_h_marker(bgr)
        if det is None:
            return

        out = TargetDetection()
        out.header = msg.header
        out.class_name = det.class_name
        out.confidence = float(det.confidence)
        out.x_min, out.y_min, out.x_max, out.y_max = det.bbox
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
