"""Build an automatically annotated YOLO steel-ball dataset.

Input layout copied from MaixCAM:

captures/
  scene_0001/
    background/*.jpg
    samples/*.jpg

The camera and background must stay fixed inside one scene. Different scenes
may use completely different colors, textures, lighting and surfaces.
"""

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
CLASS_ID = 0
CLASS_NAME = "steel_ball"

DEFAULT_MIN_RADIUS = 20
DEFAULT_MAX_RADIUS = 34
DEFAULT_ACCEPT_SCORE = 0.58
REVIEW_SCORE = 0.42

DIFF_THRESHOLD_FLOOR = 18
DIFF_THRESHOLD_CEILING = 48
MAX_CHANGED_FRAME_FRACTION = 0.18
EMPTY_CHANGED_FRAME_FRACTION = 0.0015
MAX_UNEXPLAINED_CHANGE = 0.78

MIN_INNER_CHANGE = 0.20
MIN_CHANGE_LOCALIZATION = 0.035
MIN_EDGE_SUPPORT = 0.12
MIN_INNER_STD = 10.0


@dataclass
class Candidate:
    center_x: int
    center_y: int
    radius: int
    score: float
    source: str
    inner_change: float
    ring_change: float
    edge_support: float
    texture_score: float

    def box(self, width, height):
        padded_radius = max(2, int(round(1.12 * self.radius)))
        x1 = max(0, self.center_x - padded_radius)
        y1 = max(0, self.center_y - padded_radius)
        x2 = min(width - 1, self.center_x + padded_radius)
        y2 = min(height - 1, self.center_y + padded_radius)
        return x1, y1, x2, y2


def _image_paths(directory):
    if not directory.is_dir():
        return []
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def _load_background(scene_dir):
    paths = _image_paths(scene_dir / "background")
    if not paths:
        raise RuntimeError("No background images in {}".format(scene_dir))

    images = []
    target_shape = None
    for path in paths:
        current = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if current is None:
            continue
        if target_shape is None:
            target_shape = current.shape
        if current.shape != target_shape:
            continue
        images.append(current)

    if not images:
        raise RuntimeError(
            "Background images cannot be decoded in {}".format(scene_dir)
        )

    stack = np.stack(images, axis=0)
    return np.median(stack, axis=0).astype(np.uint8), paths


def _illumination_correct(image, background):
    delta = image.astype(np.int16) - background.astype(np.int16)
    channel_shift = np.median(delta, axis=(0, 1))
    channel_shift = np.clip(channel_shift, -30.0, 30.0)
    corrected = image.astype(np.float32) - channel_shift.reshape(1, 1, 3)
    return np.clip(corrected, 0, 255).astype(np.uint8)


def _change_mask(image, background):
    corrected = _illumination_correct(image, background)
    smooth_image = cv2.GaussianBlur(corrected, (5, 5), 0)
    smooth_background = cv2.GaussianBlur(background, (5, 5), 0)
    color_difference = cv2.absdiff(smooth_image, smooth_background)
    difference = np.max(color_difference, axis=2).astype(np.uint8)

    median_noise = float(np.median(difference))
    mad = float(np.median(np.abs(difference.astype(np.float32) - median_noise)))
    threshold = int(
        round(
            min(
                DIFF_THRESHOLD_CEILING,
                max(DIFF_THRESHOLD_FLOOR, median_noise + 6.0 * mad),
            )
        )
    )
    _, mask = cv2.threshold(
        difference,
        threshold,
        255,
        cv2.THRESH_BINARY,
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1,
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2,
    )
    return corrected, difference, mask, threshold


def _circle_masks(shape, center_x, center_y, radius):
    outer_radius = max(3, int(round(1.40 * radius)))
    x1 = max(0, center_x - outer_radius)
    y1 = max(0, center_y - outer_radius)
    x2 = min(shape[1], center_x + outer_radius + 1)
    y2 = min(shape[0], center_y + outer_radius + 1)
    if x1 >= x2 or y1 >= y2:
        return None

    local_y, local_x = np.ogrid[: y2 - y1, : x2 - x1]
    local_center_x = center_x - x1
    local_center_y = center_y - y1
    distance_squared = (
        (local_x - local_center_x) ** 2
        + (local_y - local_center_y) ** 2
    )
    inner = distance_squared <= (0.82 * radius) ** 2
    ring = (
        (distance_squared >= (1.02 * radius) ** 2)
        & (distance_squared <= (1.38 * radius) ** 2)
    )
    return x1, y1, x2, y2, inner, ring


def _edge_support(edges, center_x, center_y, radius, samples=48):
    height, width = edges.shape[:2]
    hits = 0
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        x = int(round(center_x + radius * math.cos(angle)))
        y = int(round(center_y + radius * math.sin(angle)))
        x1 = max(0, x - 2)
        y1 = max(0, y - 2)
        x2 = min(width, x + 3)
        y2 = min(height, y + 3)
        if x1 < x2 and y1 < y2 and np.any(edges[y1:y2, x1:x2]):
            hits += 1
    return hits / float(samples)


def _score_circle(
    gray,
    change_mask,
    edges,
    center_x,
    center_y,
    radius,
    shape_score,
    source,
):
    masks = _circle_masks(
        gray.shape,
        center_x,
        center_y,
        radius,
    )
    if masks is None:
        return None
    x1, y1, x2, y2, inner, ring = masks
    roi_gray = gray[y1:y2, x1:x2]
    roi_change = change_mask[y1:y2, x1:x2]
    inner_pixels = roi_gray[inner]
    if inner_pixels.size < 16 or not np.any(ring):
        return None

    inner_change = float(np.mean(roi_change[inner] > 0))
    ring_change = float(np.mean(roi_change[ring] > 0))
    localization = max(0.0, inner_change - ring_change)
    edge_support = _edge_support(
        edges,
        center_x,
        center_y,
        radius,
    )
    inner_std = float(np.std(inner_pixels))
    texture_score = min(1.0, inner_std / 48.0)

    inner_mean = float(np.mean(inner_pixels))
    bright_fraction = float(
        np.mean(inner_pixels >= min(245.0, inner_mean + 22.0))
    )
    dark_fraction = float(
        np.mean(inner_pixels <= max(8.0, inner_mean - 18.0))
    )
    metal_score = math.sqrt(
        min(1.0, bright_fraction / 0.07)
        * min(1.0, dark_fraction / 0.16)
    )

    if (
        inner_change < MIN_INNER_CHANGE
        or localization < MIN_CHANGE_LOCALIZATION
        or edge_support < MIN_EDGE_SUPPORT
        or inner_std < MIN_INNER_STD
    ):
        return None

    change_score = min(1.0, inner_change / 0.65)
    localization_score = min(1.0, localization / 0.38)
    score = (
        0.28 * change_score
        + 0.22 * localization_score
        + 0.20 * edge_support
        + 0.12 * texture_score
        + 0.10 * metal_score
        + 0.08 * shape_score
    )
    return Candidate(
        center_x=int(center_x),
        center_y=int(center_y),
        radius=int(radius),
        score=float(score),
        source=source,
        inner_change=inner_change,
        ring_change=ring_change,
        edge_support=edge_support,
        texture_score=texture_score,
    )


def _contour_candidates(
    gray,
    change_mask,
    edges,
    min_radius,
    max_radius,
):
    contour_result = cv2.findContours(
        change_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    contours = contour_result[-2]
    candidates = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:80]:
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if area < 20.0 or perimeter <= 1.0:
            continue

        circularity = max(
            0.0,
            min(1.0, 4.0 * math.pi * area / (perimeter * perimeter)),
        )
        (center_x, center_y), radius_float = cv2.minEnclosingCircle(contour)
        radius = int(round(radius_float))
        if not min_radius <= radius <= max_radius:
            continue

        enclosing_area = math.pi * radius_float * radius_float
        fill = max(0.0, min(1.0, area / max(1.0, enclosing_area)))
        shape_score = 0.55 * circularity + 0.45 * fill
        candidate = _score_circle(
            gray,
            change_mask,
            edges,
            int(round(center_x)),
            int(round(center_y)),
            radius,
            shape_score,
            "contour",
        )
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def _hough_candidates(
    gray,
    change_mask,
    edges,
    min_radius,
    max_radius,
):
    blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.25,
        minDist=max(8, int(round(1.45 * min_radius))),
        param1=110,
        param2=17,
        minRadius=min_radius,
        maxRadius=max_radius,
    )
    if circles is None:
        return []

    candidates = []
    for center_x, center_y, radius in np.around(circles[0]).astype(np.int32):
        candidate = _score_circle(
            gray,
            change_mask,
            edges,
            int(center_x),
            int(center_y),
            int(radius),
            0.90,
            "hough",
        )
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def _merge_candidates(candidates):
    merged = []
    for candidate in sorted(
        candidates,
        key=lambda item: item.score,
        reverse=True,
    ):
        duplicate = False
        for current in merged:
            distance = math.hypot(
                candidate.center_x - current.center_x,
                candidate.center_y - current.center_y,
            )
            center_gate = max(
                5.0,
                1.45 * max(candidate.radius, current.radius),
            )
            if distance <= center_gate:
                duplicate = True
                break
        if not duplicate:
            merged.append(candidate)
    return merged


def _changed_pixels_explained(mask, candidates):
    changed = int(cv2.countNonZero(mask))
    if changed <= 0:
        return 1.0
    covered = np.zeros_like(mask)
    for candidate in candidates:
        cv2.circle(
            covered,
            (candidate.center_x, candidate.center_y),
            int(round(1.45 * candidate.radius)),
            255,
            -1,
        )
    explained = cv2.countNonZero(cv2.bitwise_and(mask, covered))
    return explained / float(changed)


def _annotate_frame(
    image,
    background,
    min_radius,
    max_radius,
    accept_score,
    max_changed_fraction=MAX_CHANGED_FRAME_FRACTION,
    min_explained_change=1.0 - MAX_UNEXPLAINED_CHANGE,
    single_object=False,
    center_y_range=None,
):
    corrected, difference, mask, threshold = _change_mask(
        image,
        background,
    )
    changed_fraction = cv2.countNonZero(mask) / float(mask.size)
    if changed_fraction > max_changed_fraction:
        return {
            "status": "review_large_change",
            "detections": [],
            "all_candidates": [],
            "changed_fraction": changed_fraction,
            "explained_fraction": 0.0,
            "threshold": threshold,
            "mask": mask,
            "difference": difference,
        }

    gray = cv2.cvtColor(corrected, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 45, 120)
    near_change = cv2.dilate(
        mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1,
    )
    edges = cv2.bitwise_and(edges, near_change)

    candidates = _merge_candidates(
        _contour_candidates(
            gray,
            mask,
            edges,
            min_radius,
            max_radius,
        )
        + _hough_candidates(
            gray,
            mask,
            edges,
            min_radius,
            max_radius,
        )
    )
    if center_y_range is not None:
        center_y_min, center_y_max = center_y_range
        candidates = [
            candidate
            for candidate in candidates
            if center_y_min <= candidate.center_y <= center_y_max
        ]
    if single_object and candidates:
        candidates = [max(candidates, key=lambda candidate: candidate.score)]
    accepted = [
        candidate
        for candidate in candidates
        if candidate.score >= accept_score
    ]
    uncertain = []
    for candidate in candidates:
        if not REVIEW_SCORE <= candidate.score < accept_score:
            continue
        near_accepted = any(
            math.hypot(
                candidate.center_x - current.center_x,
                candidate.center_y - current.center_y,
            )
            <= 1.75 * max(candidate.radius, current.radius)
            for current in accepted
        )
        if not near_accepted:
            uncertain.append(candidate)
    explained_fraction = _changed_pixels_explained(mask, accepted)

    if changed_fraction <= EMPTY_CHANGED_FRAME_FRACTION:
        status = "accepted_empty"
        accepted = []
    elif not accepted:
        status = "review_no_detection"
    elif uncertain:
        status = "review_uncertain"
    elif explained_fraction < min_explained_change:
        status = "review_unexplained_change"
    else:
        status = "accepted"

    return {
        "status": status,
        "detections": accepted,
        "all_candidates": candidates,
        "changed_fraction": changed_fraction,
        "explained_fraction": explained_fraction,
        "threshold": threshold,
        "mask": mask,
        "difference": difference,
    }


def _draw_preview(image, result):
    preview = image.copy()
    status = result["status"]
    accepted_ids = {id(item) for item in result["detections"]}
    for candidate in result["all_candidates"]:
        if id(candidate) in accepted_ids:
            color = (0, 255, 0)
            thickness = 2
        else:
            color = (0, 165, 255)
            thickness = 1
        x1, y1, x2, y2 = candidate.box(
            image.shape[1],
            image.shape[0],
        )
        cv2.rectangle(preview, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            preview,
            "{:.2f} {}".format(candidate.score, candidate.source[0]),
            (x1, max(14, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )

    status_color = (
        (0, 255, 0) if status.startswith("accepted") else (0, 0, 255)
    )
    cv2.rectangle(
        preview,
        (0, 0),
        (image.shape[1], 26),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        preview,
        "{} balls:{} change:{:.1f}% explained:{:.0f}%".format(
            status,
            len(result["detections"]),
            100.0 * result["changed_fraction"],
            100.0 * result["explained_fraction"],
        ),
        (6, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        status_color,
        1,
        cv2.LINE_AA,
    )
    return preview


def _yolo_lines(candidates, width, height):
    lines = []
    for candidate in candidates:
        x1, y1, x2, y2 = candidate.box(width, height)
        box_width = max(1, x2 - x1 + 1)
        box_height = max(1, y2 - y1 + 1)
        center_x = x1 + 0.5 * box_width
        center_y = y1 + 0.5 * box_height
        lines.append(
            "{} {:.6f} {:.6f} {:.6f} {:.6f}".format(
                CLASS_ID,
                center_x / width,
                center_y / height,
                box_width / width,
                box_height / height,
            )
        )
    return lines


def _stable_fraction(text):
    digest = hashlib.sha1(text.encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], byteorder="big", signed=False)
    return value / float((1 << 64) - 1)


def _scene_splits(scene_dirs, validation_fraction):
    if len(scene_dirs) >= 3:
        splits = {}
        for scene_dir in scene_dirs:
            value = _stable_fraction(scene_dir.name)
            splits[scene_dir.name] = (
                "val" if value < validation_fraction else "train"
            )
        if "val" not in splits.values():
            splits[scene_dirs[-1].name] = "val"
        if "train" not in splits.values():
            splits[scene_dirs[0].name] = "train"
        return splits, True
    return {}, False


def _next_output_dir(output_root):
    output_root.mkdir(parents=True, exist_ok=True)
    prefix = "dataset_run_"
    existing_indices = []
    for path in output_root.iterdir():
        if not path.is_dir() or not path.name.startswith(prefix):
            continue
        suffix = path.name[len(prefix) :]
        if suffix.isdigit():
            existing_indices.append(int(suffix))

    index = max(existing_indices, default=0) + 1
    if index >= 10000:
        raise RuntimeError(
            "No free dataset_run directory in {}".format(output_root)
        )

    candidate = output_root / "{}{:04d}".format(prefix, index)
    candidate.mkdir()
    return candidate


def _write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _copy_accepted(
    source_path,
    image,
    result,
    output_dir,
    split,
    output_stem,
):
    image_dir = output_dir / "images" / split
    label_dir = output_dir / "labels" / split
    preview_dir = output_dir / "preview" / split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)

    output_image = image_dir / (output_stem + source_path.suffix.lower())
    cv2.imwrite(str(output_image), image)
    lines = _yolo_lines(
        result["detections"],
        image.shape[1],
        image.shape[0],
    )
    _write_text(
        label_dir / (output_stem + ".txt"),
        "\n".join(lines) + ("\n" if lines else ""),
    )
    cv2.imwrite(
        str(preview_dir / (output_stem + ".jpg")),
        _draw_preview(image, result),
    )


def _copy_review(
    source_path,
    image,
    result,
    output_dir,
    output_stem,
):
    review_dir = output_dir / "review" / result["status"]
    review_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(
        str(review_dir / (output_stem + ".jpg")),
        _draw_preview(image, result),
    )


def _add_known_negatives(
    scene_dir,
    background_paths,
    output_dir,
    split,
    roi,
    stride=5,
):
    count = 0
    for index, source_path in enumerate(background_paths):
        if index % stride != 0:
            continue
        image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        if roi is not None:
            x, y, width, height = roi
            image = image[y : y + height, x : x + width]
        stem = "{}__negative_{:04d}".format(scene_dir.name, index)
        result = {
            "status": "accepted_empty",
            "detections": [],
            "all_candidates": [],
            "changed_fraction": 0.0,
            "explained_fraction": 1.0,
        }
        _copy_accepted(
            source_path,
            image,
            result,
            output_dir,
            split,
            stem,
        )
        count += 1
    return count


def _parse_roi(value):
    try:
        x, y, width, height = [int(item) for item in value.split(",")]
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height")
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError(
            "ROI coordinates must be non-negative and size must be positive"
        )
    return x, y, width, height


def _parse_center_y_range(value):
    try:
        minimum, maximum = [int(item) for item in value.split(",")]
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(
            "center-y-range must be minimum,maximum"
        )
    if minimum < 0 or maximum <= minimum:
        raise argparse.ArgumentTypeError(
            "center-y-range must satisfy 0 <= minimum < maximum"
        )
    return minimum, maximum


def _arguments():
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Auto-annotate MaixCAM steel-ball captures",
    )
    parser.add_argument(
        "--captures",
        type=Path,
        default=project_dir / "captures",
        help="Directory containing scene_*/background and samples",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_dir / "datasets",
        help="A new dataset_run_XXXX directory is created here",
    )
    parser.add_argument(
        "--scene",
        action="append",
        dest="scenes",
        help=(
            "Only process this scene directory name. "
            "Repeat --scene to select multiple scenes."
        ),
    )
    parser.add_argument(
        "--roi",
        type=_parse_roi,
        help=(
            "Crop every selected scene to x,y,width,height before detection "
            "and dataset export. Use only when the same ROI is valid for all "
            "selected scenes."
        ),
    )
    parser.add_argument(
        "--min-radius",
        type=int,
        default=DEFAULT_MIN_RADIUS,
    )
    parser.add_argument(
        "--max-radius",
        type=int,
        default=DEFAULT_MAX_RADIUS,
    )
    parser.add_argument(
        "--accept-score",
        type=float,
        default=DEFAULT_ACCEPT_SCORE,
    )
    parser.add_argument(
        "--max-changed-fraction",
        type=float,
        default=MAX_CHANGED_FRAME_FRACTION,
        help=(
            "Reject frames with more foreground change than this fraction; "
            "lower it to exclude hands in fixed-camera scenes."
        ),
    )
    parser.add_argument(
        "--min-explained-change",
        type=float,
        default=1.0 - MAX_UNEXPLAINED_CHANGE,
        help=(
            "Require accepted boxes to cover at least this fraction of the "
            "foreground change."
        ),
    )
    parser.add_argument(
        "--validation-fraction",
        type=float,
        default=0.20,
    )
    parser.add_argument(
        "--background-stride",
        type=int,
        default=5,
        help=(
            "Use every Nth empty-background image as a known negative. "
            "Set to 1 to keep all captured empty frames."
        ),
    )
    parser.add_argument(
        "--single-object",
        action="store_true",
        help=(
            "Keep only the strongest circle candidate in each frame. "
            "Use only when one steel ball is present during sample capture."
        ),
    )
    parser.add_argument(
        "--center-y-range",
        type=_parse_center_y_range,
        help=(
            "Accept circle centers only within minimum,maximum image rows. "
            "Coordinates are evaluated after --roi cropping."
        ),
    )
    return parser.parse_args()


def main():
    args = _arguments()
    if args.min_radius < 2 or args.max_radius <= args.min_radius:
        raise ValueError("Invalid min/max radius")
    if not 0.0 < args.validation_fraction < 0.5:
        raise ValueError("validation-fraction must be between 0 and 0.5")
    if not 0.0 < args.max_changed_fraction <= 1.0:
        raise ValueError("max-changed-fraction must be in (0, 1]")
    if not 0.0 <= args.min_explained_change <= 1.0:
        raise ValueError("min-explained-change must be in [0, 1]")
    if args.background_stride < 1:
        raise ValueError("background-stride must be at least 1")

    scene_dirs = sorted(
        path
        for path in args.captures.glob("scene_*")
        if path.is_dir()
    )
    if args.scenes:
        requested_scenes = set(args.scenes)
        available_scenes = {path.name for path in scene_dirs}
        missing_scenes = requested_scenes - available_scenes
        if missing_scenes:
            raise RuntimeError(
                "Requested scene directories not found: {}".format(
                    ", ".join(sorted(missing_scenes))
                )
            )
        scene_dirs = [
            path
            for path in scene_dirs
            if path.name in requested_scenes
        ]
    if not scene_dirs:
        raise RuntimeError(
            "No scene_* directories found under {}".format(args.captures)
        )

    output_dir = _next_output_dir(args.output_root)
    scene_split, scene_level_split = _scene_splits(
        scene_dirs,
        args.validation_fraction,
    )
    report = {
        "captures": str(args.captures.resolve()),
        "output": str(output_dir.resolve()),
        "settings": {
            "min_radius": args.min_radius,
            "max_radius": args.max_radius,
            "accept_score": args.accept_score,
            "max_changed_fraction": args.max_changed_fraction,
            "min_explained_change": args.min_explained_change,
            "validation_fraction": args.validation_fraction,
            "scene_filter": args.scenes,
            "roi": args.roi,
            "single_object": args.single_object,
            "center_y_range": args.center_y_range,
            "background_stride": args.background_stride,
        },
        "scene_level_split": scene_level_split,
        "counts": {
            "accepted": 0,
            "accepted_empty": 0,
            "known_negative": 0,
            "review": 0,
            "unreadable": 0,
            "boxes": 0,
        },
        "status_counts": {},
        "scenes": {},
    }
    annotation_rows = []

    for scene_dir in scene_dirs:
        background, background_paths = _load_background(scene_dir)
        if args.roi is not None:
            x, y, width, height = args.roi
            if (
                x + width > background.shape[1]
                or y + height > background.shape[0]
            ):
                raise ValueError(
                    "ROI {} exceeds image size {}x{} in {}".format(
                        args.roi,
                        background.shape[1],
                        background.shape[0],
                        scene_dir.name,
                    )
                )
            background = background[y : y + height, x : x + width]
        sample_paths = _image_paths(scene_dir / "samples")
        if scene_level_split:
            split = scene_split[scene_dir.name]
        else:
            split = None

        scene_report = {
            "samples": len(sample_paths),
            "background_images": len(background_paths),
            "split": split or "per-image",
        }
        report["scenes"][scene_dir.name] = scene_report

        negative_split = split or "train"
        negative_count = _add_known_negatives(
            scene_dir,
            background_paths,
            output_dir,
            negative_split,
            args.roi,
            args.background_stride,
        )
        report["counts"]["known_negative"] += negative_count

        for sample_index, sample_path in enumerate(sample_paths):
            image = cv2.imread(str(sample_path), cv2.IMREAD_COLOR)
            if image is None:
                report["counts"]["unreadable"] += 1
                continue
            if args.roi is not None:
                x, y, width, height = args.roi
                image = image[y : y + height, x : x + width]
            if image.shape != background.shape:
                report["counts"]["unreadable"] += 1
                continue

            if split is None:
                split_value = _stable_fraction(
                    "{}:{}".format(scene_dir.name, sample_path.name)
                )
                frame_split = (
                    "val"
                    if split_value < args.validation_fraction
                    else "train"
                )
            else:
                frame_split = split

            result = _annotate_frame(
                image,
                background,
                args.min_radius,
                args.max_radius,
                args.accept_score,
                args.max_changed_fraction,
                args.min_explained_change,
                args.single_object,
                args.center_y_range,
            )
            status = result["status"]
            report["status_counts"][status] = (
                report["status_counts"].get(status, 0) + 1
            )
            output_stem = "{}__{:06d}".format(
                scene_dir.name,
                sample_index,
            )

            if status.startswith("accepted"):
                _copy_accepted(
                    sample_path,
                    image,
                    result,
                    output_dir,
                    frame_split,
                    output_stem,
                )
                if status == "accepted_empty":
                    report["counts"]["accepted_empty"] += 1
                else:
                    report["counts"]["accepted"] += 1
                report["counts"]["boxes"] += len(result["detections"])
            else:
                _copy_review(
                    sample_path,
                    image,
                    result,
                    output_dir,
                    output_stem,
                )
                report["counts"]["review"] += 1

            for detection_index, detection in enumerate(
                result["detections"]
            ):
                annotation_rows.append(
                    {
                        "scene": scene_dir.name,
                        "image": sample_path.name,
                        "split": frame_split,
                        "status": status,
                        "detection": detection_index,
                        "center_x": detection.center_x,
                        "center_y": detection.center_y,
                        "radius": detection.radius,
                        "score": "{:.4f}".format(detection.score),
                        "source": detection.source,
                        "inner_change": "{:.4f}".format(
                            detection.inner_change
                        ),
                        "ring_change": "{:.4f}".format(
                            detection.ring_change
                        ),
                        "edge_support": "{:.4f}".format(
                            detection.edge_support
                        ),
                    }
                )

    dataset_yaml = (
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: {}\n"
    ).format(CLASS_NAME)
    _write_text(output_dir / "data.yaml", dataset_yaml)
    _write_text(
        output_dir / "report.json",
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    )

    csv_path = output_dir / "annotations.csv"
    fieldnames = [
        "scene",
        "image",
        "split",
        "status",
        "detection",
        "center_x",
        "center_y",
        "radius",
        "score",
        "source",
        "inner_change",
        "ring_change",
        "edge_support",
    ]
    with csv_path.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(annotation_rows)

    print("Dataset created: {}".format(output_dir))
    print(json.dumps(report["counts"], ensure_ascii=False, indent=2))
    if not scene_level_split:
        print(
            "WARNING: fewer than 3 scenes; train/val split is per image. "
            "Capture more scenes before using validation metrics."
        )
    print(
        "Review images are excluded from YOLO labels and saved under: {}".format(
            output_dir / "review"
        )
    )


if __name__ == "__main__":
    main()
