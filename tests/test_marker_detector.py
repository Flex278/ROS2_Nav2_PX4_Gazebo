"""Тесты детекции H-маркера (OpenCV, синтетические кадры)."""
import cv2
import numpy as np

from _common import ROOT  # noqa: F401
from uav_perception.detection import detect_h_marker


def _make_h_image():
    img = np.full((160, 160, 3), 255, np.uint8)
    cv2.rectangle(img, (40, 30), (60, 130), (0, 0, 0), -1)
    cv2.rectangle(img, (100, 30), (120, 130), (0, 0, 0), -1)
    cv2.rectangle(img, (40, 75), (120, 95), (0, 0, 0), -1)
    return img


def _iou(a, b):
    ix = max(a[0], b[0]); iy = max(a[1], b[1])
    ax = min(a[2], b[2]); ay = min(a[3], b[3])
    inter = max(0, ax - ix) * max(0, ay - iy)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + ab - inter)


def test_detect_h_marker():
    det = detect_h_marker(_make_h_image())
    assert det is not None, 'H-маркер не найден'
    assert det.class_name == 'h_marker'
    assert _iou(det.bbox, (40, 30, 120, 130)) > 0.7, det.bbox


def test_no_marker_on_blank():
    det = detect_h_marker(np.full((100, 100, 3), 128, np.uint8))
    assert det is None


def test_grayscale_supported():
    det = detect_h_marker(cv2.cvtColor(_make_h_image(), cv2.COLOR_BGR2GRAY))
    assert det is not None and det.class_name == 'h_marker'


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
