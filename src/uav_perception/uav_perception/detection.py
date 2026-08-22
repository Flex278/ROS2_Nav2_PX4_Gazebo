"""Чистые функции детекции (numpy + OpenCV) — тестируются локально.

Здесь нет rclpy/uav_msgs, только логика, чтобы её можно было прогонять в unit-тестах.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    """Результат детекции."""

    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x_min, y_min, x_max, y_max


def detect_h_marker(bgr: np.ndarray, min_area: float = 0.0005) -> Detection | None:
    """Найти H-маркер на кадре (BGR).

    Подход: бинаризация (H тёмный на светлом фоне) -> наибольший контур ->
    оценка формы «H» (две ножки + перекладина). Возвращает None, если не найден.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    _, bin_img = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    img_area = float(gray.shape[0] * gray.shape[1])
    best: tuple[int, int, int, int, float] | None = None
    for cnt in contours:
        if cv2.contourArea(cnt) < min_area * img_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if w < 8 or h < 8:
            continue
        aspect = w / float(h)
        if not (0.4 <= aspect <= 2.5):
            continue
        roi = bin_img[y:y + h, x:x + w]
        score = _h_shape_score(roi)
        fill = float(np.count_nonzero(roi)) / (w * h)
        conf = 0.5 * score + 0.5 * min(1.0, fill * 2.0)
        if best is None or conf > best[4]:
            best = (x, y, x + w, y + h, conf)

    if best is None or best[4] < 0.35:
        return None
    return Detection(
        'h_marker', float(best[4]),
        (int(best[0]), int(best[1]), int(best[2]), int(best[3])))


def _h_shape_score(binary_roi: np.ndarray) -> float:
    """Оценка 0..1 того, что бинарная ROI содержит форму буквы H."""
    h, w = binary_roi.shape
    if h < 6 or w < 6:
        return 0.0
    top = binary_roi[: h // 3, :]
    mid = binary_roi[h // 3: 2 * h // 3, :]
    bot = binary_roi[2 * h // 3:, :]
    lw = max(1, int(w * 0.4))
    rw = max(1, int(w * 0.6))

    def fill(region: np.ndarray) -> float:
        return float(np.count_nonzero(region)) / max(1, region.size)

    tl = fill(top[:, :lw])
    tr = fill(top[:, rw:])
    tc = fill(top[:, lw:rw])
    ml = fill(mid[:, :lw])
    mr = fill(mid[:, rw:])
    mc = fill(mid[:, lw:rw])
    bl = fill(bot[:, :lw])
    br = fill(bot[:, rw:])
    bc = fill(bot[:, lw:rw])

    legs_ok = min(tl, tr, bl, br) > 0.45   # четыре ножки заполнены
    crossbar_ok = mc > 0.45                 # перекладина заполнена
    gaps_ok = max(tc, bc) < 0.30            # верх/низ по центру пустые
    return (legs_ok + crossbar_ok + gaps_ok) / 3.0
