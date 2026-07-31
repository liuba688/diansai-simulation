"""A4 黑色矩形靶框检测。"""

import cv2
import numpy as np

import config


class TargetDetection:
    def __init__(
        self,
        found=False,
        corners=None,
        center=(0, 0),
        width=0,
        height=0,
        area=0.0,
        confidence=0,
        boundary_edges=0,
        precise=True,
    ):
        self.found = found
        self.corners = corners
        self.center = center
        self.width = width
        self.height = height
        self.area = area
        self.confidence = confidence
        self.boundary_edges = boundary_edges
        self.precise = precise


def _order_points(points):
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    point_sum = points.sum(axis=1)
    point_diff = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(point_sum)]
    ordered[1] = points[np.argmin(point_diff)]
    ordered[2] = points[np.argmax(point_sum)]
    ordered[3] = points[np.argmax(point_diff)]
    return ordered


def _distance(point_a, point_b):
    delta = point_a - point_b
    return float(np.sqrt(delta[0] * delta[0] + delta[1] * delta[1]))


def _diagonal_intersection(corners):
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


def _max_angle_cosine(corners):
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


def _count_boundary_edges(x, y, width, height):
    margin = 3
    count = 0
    if x <= margin:
        count += 1
    if y <= margin:
        count += 1
    if x + width >= config.FRAME_WIDTH - margin:
        count += 1
    if y + height >= config.FRAME_HEIGHT - margin:
        count += 1
    return count


class TargetDetector:
    def __init__(self):
        self._kernel = None
        if config.USE_MORPH_CLOSE:
            size = config.MORPH_KERNEL_SIZE
            self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))

    def detect(self, image_bgr):
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, config.GAUSSIAN_KERNEL, 0)
        if self._kernel is not None:
            gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, self._kernel)
        if config.BORDER_SIZE > 0:
            gray = cv2.copyMakeBorder(
                gray,
                config.BORDER_SIZE,
                config.BORDER_SIZE,
                config.BORDER_SIZE,
                config.BORDER_SIZE,
                cv2.BORDER_CONSTANT,
                value=0,
            )

        edges = cv2.Canny(gray, config.CANNY_LOW, config.CANNY_HIGH)
        contour_result = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]

        frame_area = float(config.FRAME_WIDTH * config.FRAME_HEIGHT)
        min_area = config.MIN_AREA_RATIO * frame_area
        max_area = config.MAX_AREA_RATIO * frame_area
        large_contours = []
        for contour in contours:
            raw_area = abs(float(cv2.contourArea(contour)))
            if 0.65 * min_area <= raw_area <= 1.05 * max_area:
                large_contours.append((raw_area, contour))
        large_contours.sort(key=lambda item: item[0], reverse=True)

        candidates = []
        for _, contour in large_contours[: config.MAX_CONTOURS_TO_CHECK]:
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 1.0:
                continue
            approximation = cv2.approxPolyDP(
                contour, config.APPROX_EPSILON * perimeter, True
            )
            if len(approximation) != 4 or not cv2.isContourConvex(approximation):
                continue

            corners = approximation.reshape(4, 2).astype(np.float32)
            corners -= config.BORDER_SIZE
            corners[:, 0] = np.clip(corners[:, 0], 0, config.FRAME_WIDTH - 1)
            corners[:, 1] = np.clip(corners[:, 1], 0, config.FRAME_HEIGHT - 1)
            corners = _order_points(corners)
            polygon_area = abs(float(cv2.contourArea(corners)))
            if polygon_area < min_area or polygon_area > max_area:
                continue

            top_width = _distance(corners[0], corners[1])
            bottom_width = _distance(corners[3], corners[2])
            left_height = _distance(corners[0], corners[3])
            right_height = _distance(corners[1], corners[2])
            average_width = 0.5 * (top_width + bottom_width)
            average_height = 0.5 * (left_height + right_height)
            short_side = min(average_width, average_height)
            long_side = max(average_width, average_height)
            if short_side < 6.0:
                continue
            aspect_ratio = long_side / short_side
            if not config.MIN_ASPECT_RATIO <= aspect_ratio <= config.MAX_ASPECT_RATIO:
                continue

            angle_cosine = _max_angle_cosine(corners)
            if angle_cosine > config.MAX_ANGLE_COSINE:
                continue
            rotated_rect = cv2.minAreaRect(corners.astype(np.float32))
            rect_width, rect_height = rotated_rect[1]
            rect_area = max(1.0, float(rect_width) * float(rect_height))
            rectangularity = polygon_area / rect_area
            if rectangularity < config.MIN_RECTANGULARITY:
                continue

            x, y, width, height = cv2.boundingRect(corners.astype(np.int32))
            boundary_edges = _count_boundary_edges(x, y, width, height)
            if boundary_edges > 1:
                continue

            center = _diagonal_intersection(corners)
            center_x = int(round(float(center[0])))
            center_y = int(round(float(center[1])))
            area_score = min(1.0, polygon_area / (0.30 * frame_area))
            aspect_score = max(
                0.0,
                1.0 - abs(aspect_ratio - config.TARGET_ASPECT_RATIO) / 0.80,
            )
            angle_score = max(
                0.0, 1.0 - angle_cosine / config.MAX_ANGLE_COSINE
            )
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
            ranking = confidence + 80.0 * polygon_area / frame_area
            ranking -= 8.0 * boundary_edges
            candidates.append(
                (
                    ranking,
                    TargetDetection(
                        True,
                        corners,
                        (center_x, center_y),
                        int(round(average_width)),
                        int(round(average_height)),
                        polygon_area,
                        max(0, min(100, confidence)),
                        boundary_edges,
                        boundary_edges == 0,
                    ),
                )
            )

        if not candidates:
            return TargetDetection()
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]
