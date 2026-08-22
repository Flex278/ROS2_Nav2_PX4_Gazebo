"""Мультиклассовый детектор объектов на базе YOLO (ultralytics).

Подписывается на /camera/image_raw (sensor_msgs/Image),
публикует /detections (uav_msgs/TargetDetection) для каждого найденного объекта
из списка классов (параметр `classes`, по умолчанию ['person']). class_name
берётся из словаря COCO как есть: 'person', 'car', 'traffic light', 'stop sign'.

ultralytics импортируется лениво — он живёт в отдельном venv (см. setup_venv.sh).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from uav_msgs.msg import TargetDetection
from uav_perception.cv_utils import image_to_numpy, to_rgb


class HumanDetectorNode(Node):
    """Нода мультиклассовой детекции объектов (YOLO)."""

    def __init__(self) -> None:
        super().__init__('human_detector')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        # Список COCO-классов, которые публикуем (по умолчанию — только человек).
        self.declare_parameter('classes', ['person'])

        camera = self.get_parameter('camera_topic').value
        det_topic = self.get_parameter('detection_topic').value
        self.create_subscription(Image, camera, self._on_image, 10)
        self._pub = self.create_publisher(TargetDetection, det_topic, 10)

        self._conf = float(self.get_parameter('conf_threshold').value)
        # Имена классов COCO как есть: 'person', 'car', 'traffic light', 'stop sign'.
        self._classes = set(str(c) for c in self.get_parameter('classes').value)
        self._model = None
        self.get_logger().info(
            f'HumanDetectorNode инициализирована, classes={sorted(self._classes)}')

    def _load_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.get_parameter('yolo_model').value)
        return self._model

    def _on_image(self, msg: Image) -> None:
        try:
            model = self._load_model()
            rgb = to_rgb(image_to_numpy(msg), msg.encoding)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'YOLO недоступен: {e}', once=True)
            return

        results = model.predict(rgb, conf=self._conf, verbose=False)
        for r in results:
            names = r.names
            for box in r.boxes:
                cls = names[int(box.cls[0])]
                if cls not in self._classes:
                    continue
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                out = TargetDetection()
                out.header = msg.header
                out.class_name = cls
                out.confidence = float(box.conf[0])
                out.x_min, out.y_min, out.x_max, out.y_max = x1, y1, x2, y2
                self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
