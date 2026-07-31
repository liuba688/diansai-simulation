"""2025 E 第一阶段：A4 黑色矩形靶框识别（单文件刷新版）。"""

import time

import cv2
import numpy as np
from maix import app, camera, display, image


FRAME_WIDTH = 240
FRAME_HEIGHT = 180
DETECT_EVERY_N_FRAMES = 2
MAX_CONTOURS_TO_CHECK = 12

GAUSSIAN_KERNEL = (3, 3)
MORPH_KERNEL_SIZE = 3
USE_MORPH_CLOSE = False
BORDER_SIZE = 0
CANNY_LOW = 50
CANNY_HIGH = 150
APPROX_EPSILON = 0.025
MIN_AREA_RATIO = 0.02
MAX_AREA_RATIO = 0.95
MIN_ASPECT_RATIO = 1.00
MAX_ASPECT_RATIO = 2.20
TARGET_ASPECT_RATIO = 297.0 / 210.0
MAX_ANGLE_COSINE = 0.42
MIN_RECTANGULARITY = 0.55

EMA_ALPHA = 0.35
HOLD_FRAMES = 2
LOST_FRAMES = 5
LOG_INTERVAL_S = 0.5

_clock = getattr(time, "monotonic", time.time)


def order_points(points):
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    point_sum = points.sum(axis=1)
    point_diff = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(point_sum)]
    ordered[1] = points[np.argmin(point_diff)]
    ordered[2] = points[np.argmax(point_sum)]
    ordered[3] = points[np.argmax(point_diff)]
    return ordered


def distance(point_a, point_b):
    delta = point_a - point_b
    return float(np.sqrt(delta[0] * delta[0] + delta[1] * delta[1]))


def diagonal_intersection(corners):
    point_a = corners[0]
    vector_a = corners[2] - corners[0]
    point_b = corners[1]
    vector_b = corners[3] - corners[1]
    denominator = (
        float(vector_a[0]) * float(vector_b[1])
        - float(vector_a[1]) * float(vector_b[0])
    )
    if abs(denominator) <= 1e-6:
        return np.mean(corners, axis=0)
    offset = point_b - point_a
    scale = (
        float(offset[0]) * float(vector_b[1])
        - float(offset[1]) * float(vector_b[0])
    ) / denominator
    return point_a + scale * vector_a


def max_angle_cosine(corners):
    maximum = 0.0
    for index in range(4):
        previous_point = corners[(index - 1) % 4]
        current_point = corners[index]
        next_point = corners[(index + 1) % 4]
        vector_a = previous_point - current_point
        vector_b = next_point - current_point
        denominator = float(np.linalg.norm(vector_a) * np.linalg.norm(vector_b))
        if denominator <= 1e-6:
            return 1.0
        cosine = abs(float(np.dot(vector_a, vector_b)) / denominator)
        maximum = max(maximum, cosine)
    return maximum


def count_boundary_edges(x, y, width, height):
    margin = 3
    count = 0
    if x <= margin:
        count += 1
    if y <= margin:
        count += 1
    if x + width >= FRAME_WIDTH - margin:
        count += 1
    if y + height >= FRAME_HEIGHT - margin:
        count += 1
    return count


class Detection:
    def __init__(
        self,
        found=False,
        corners=None,
        center=(0, 0),
        confidence=0,
        precise=False,
    ):
        self.found = found
        self.corners = corners
        self.center = center
        self.confidence = confidence
        self.precise = precise


class TargetDetector:
    def __init__(self):
        self.kernel = None
        if USE_MORPH_CLOSE:
            self.kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
            )

    def detect(self, image_bgr):
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, GAUSSIAN_KERNEL, 0)
        if self.kernel is not None:
            gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, self.kernel)
        if BORDER_SIZE > 0:
            extended = cv2.copyMakeBorder(
                gray,
                BORDER_SIZE,
                BORDER_SIZE,
                BORDER_SIZE,
                BORDER_SIZE,
                cv2.BORDER_CONSTANT,
                value=0,
            )
        else:
            extended = gray
        edges = cv2.Canny(extended, CANNY_LOW, CANNY_HIGH)
        contour_result = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]

        frame_area = float(FRAME_WIDTH * FRAME_HEIGHT)
        min_area = MIN_AREA_RATIO * frame_area
        max_area = MAX_AREA_RATIO * frame_area
        large_contours = []
        for contour in contours:
            raw_area = abs(float(cv2.contourArea(contour)))
            if 0.65 * min_area <= raw_area <= 1.05 * max_area:
                large_contours.append((raw_area, contour))
        large_contours.sort(key=lambda item: item[0], reverse=True)

        candidates = []
        for _, contour in large_contours[:MAX_CONTOURS_TO_CHECK]:
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 1.0:
                continue
            approximation = cv2.approxPolyDP(
                contour, APPROX_EPSILON * perimeter, True
            )
            if len(approximation) != 4 or not cv2.isContourConvex(approximation):
                continue

            corners = approximation.reshape(4, 2).astype(np.float32)
            corners -= BORDER_SIZE
            corners[:, 0] = np.clip(corners[:, 0], 0, FRAME_WIDTH - 1)
            corners[:, 1] = np.clip(corners[:, 1], 0, FRAME_HEIGHT - 1)
            corners = order_points(corners)

            polygon_area = abs(float(cv2.contourArea(corners)))
            area_ratio = polygon_area / frame_area
            if not (MIN_AREA_RATIO <= area_ratio <= MAX_AREA_RATIO):
                continue

            top_width = distance(corners[0], corners[1])
            bottom_width = distance(corners[3], corners[2])
            left_height = distance(corners[0], corners[3])
            right_height = distance(corners[1], corners[2])
            average_width = 0.5 * (top_width + bottom_width)
            average_height = 0.5 * (left_height + right_height)
            short_side = min(average_width, average_height)
            long_side = max(average_width, average_height)
            if short_side < 8.0:
                continue
            aspect_ratio = long_side / short_side
            if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO):
                continue

            angle_cosine = max_angle_cosine(corners)
            if angle_cosine > MAX_ANGLE_COSINE:
                continue

            rotated_rect = cv2.minAreaRect(corners.astype(np.float32))
            rect_width, rect_height = rotated_rect[1]
            rect_area = max(1.0, float(rect_width) * float(rect_height))
            rectangularity = polygon_area / rect_area
            if rectangularity < MIN_RECTANGULARITY:
                continue

            x, y, width, height = cv2.boundingRect(corners.astype(np.int32))
            boundary_edges = count_boundary_edges(x, y, width, height)
            if boundary_edges > 1:
                continue

            center = diagonal_intersection(corners)
            center_xy = (
                int(round(float(center[0]))),
                int(round(float(center[1]))),
            )
            aspect_score = max(
                0.0,
                1.0 - abs(aspect_ratio - TARGET_ASPECT_RATIO) / 0.80,
            )
            angle_score = max(0.0, 1.0 - angle_cosine / MAX_ANGLE_COSINE)
            area_score = min(1.0, area_ratio / 0.30)
            confidence = int(
                round(
                    100.0
                    * (
                        0.35 * area_score
                        + 0.25 * aspect_score
                        + 0.25 * angle_score
                        + 0.15 * min(1.0, rectangularity)
                    )
                )
            )
            ranking = confidence + 80.0 * area_ratio - 8.0 * boundary_edges
            candidates.append(
                (
                    ranking,
                    Detection(
                        True,
                        corners,
                        center_xy,
                        max(0, min(100, confidence)),
                        boundary_edges == 0,
                    ),
                )
            )

        if not candidates:
            return Detection()
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]


class Tracker:
    def __init__(self):
        self.initialized = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.corners = None
        self.confidence = 0.0
        self.missed = 0

    def update(self, detection):
        if detection.found and not detection.precise:
            self.missed += 1
            if self.missed >= LOST_FRAMES:
                self.initialized = False
            return "SEARCH", detection.center, detection.corners, detection.confidence

        if detection.found:
            new_x, new_y = detection.center
            if not self.initialized:
                self.center_x = float(new_x)
                self.center_y = float(new_y)
                self.confidence = float(detection.confidence)
                self.initialized = True
            else:
                self.center_x += EMA_ALPHA * (new_x - self.center_x)
                self.center_y += EMA_ALPHA * (new_y - self.center_y)
                self.confidence += EMA_ALPHA * (
                    detection.confidence - self.confidence
                )
            self.corners = detection.corners
            self.missed = 0
            return (
                "TARGET",
                (int(round(self.center_x)), int(round(self.center_y))),
                self.corners,
                int(round(self.confidence)),
            )

        self.missed += 1
        if self.initialized and self.missed <= HOLD_FRAMES:
            return (
                "HOLD",
                (int(round(self.center_x)), int(round(self.center_y))),
                self.corners,
                int(round(self.confidence)),
            )
        if self.missed >= LOST_FRAMES:
            self.initialized = False
        return "LOST", (0, 0), None, 0


def draw_cross(image_bgr, x, y, size, color, thickness=1):
    cv2.line(image_bgr, (x - size, y), (x + size, y), color, thickness)
    cv2.line(image_bgr, (x, y - size), (x, y + size), color, thickness)


def main():
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
    disp = display.Display()
    detector = TargetDetector()
    tracker = Tracker()
    aim_x = FRAME_WIDTH // 2
    aim_y = FRAME_HEIGHT // 2
    last_time = _clock()
    last_log = 0.0
    fps = 0.0
    frame_count = 0
    last_result = ("LOST", (0, 0), None, 0)

    print("2025 E phase 1: A4 black-frame detection")
    print("Laser and motor output are disabled")

    try:
        while not app.need_exit():
            frame = cam.read()
            image_bgr = image.image2cv(frame, ensure_bgr=True, copy=False)
            if frame_count % DETECT_EVERY_N_FRAMES == 0:
                last_result = tracker.update(detector.detect(image_bgr))
            status, center, corners, confidence = last_result

            now = _clock()
            elapsed = now - last_time
            last_time = now
            if elapsed > 1e-6:
                instant_fps = 1.0 / elapsed
                fps = instant_fps if fps <= 0.0 else 0.9 * fps + 0.1 * instant_fps

            draw_cross(image_bgr, aim_x, aim_y, 10, (0, 255, 0), 2)
            if corners is not None:
                color = {
                    "TARGET": (0, 0, 255),
                    "HOLD": (0, 255, 255),
                    "SEARCH": (255, 128, 0),
                }.get(status, (255, 255, 255))
                polygon = np.asarray(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(image_bgr, [polygon], True, color, 2)
                cv2.circle(image_bgr, center, 4, (255, 0, 0), -1)

            dx = center[0] - aim_x if status != "LOST" else 0
            dy = center[1] - aim_y if status != "LOST" else 0
            cv2.putText(
                image_bgr,
                "{} dx:{:+d} dy:{:+d} conf:{}".format(
                    status, dx, dy, confidence
                ),
                (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 0) if status == "TARGET" else (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                image_bgr,
                "fps:{:.1f}".format(fps),
                (6, FRAME_HEIGHT - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            if now - last_log >= LOG_INTERVAL_S:
                print(
                    "{} x={} y={} dx={} dy={} conf={} fps={:.1f}".format(
                        status,
                        center[0],
                        center[1],
                        dx,
                        dy,
                        confidence,
                        fps,
                    )
                )
                last_log = now

            disp.show(image.cv2image(image_bgr, copy=False))
            frame_count += 1
    finally:
        cam.close()


if __name__ == "__main__":
    main()
