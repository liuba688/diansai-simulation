"""Build conservative YOLO labels using a teacher model plus circle checks.

This fallback is intended for captures where the camera moved and background
difference cannot be trusted. A frame is accepted only when every teacher box
matches a strong classical steel-ball candidate and vice versa. Ambiguous
frames are copied to review/ and never become training labels.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import build_dataset as classical


CLASS_ID = 0
CLASS_NAME = "steel_ball"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass
class TeacherBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float

    @property
    def center_x(self):
        return 0.5 * (self.x1 + self.x2)

    @property
    def center_y(self):
        return 0.5 * (self.y1 + self.y2)

    @property
    def radius(self):
        return 0.25 * (
            (self.x2 - self.x1) + (self.y2 - self.y1)
        )


def _image_paths(directory):
    if not directory.is_dir():
        return []
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def _parse_roi(value):
    try:
        x, y, width, height = (
            int(part.strip()) for part in value.split(",")
        )
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(
            "ROI must be x,y,width,height"
        )
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError(
            "ROI coordinates must be non-negative with positive size"
        )
    return x, y, width, height


def _inside_roi(center_x, center_y, roi):
    if roi is None:
        return True
    x, y, width, height = roi
    return (
        x <= center_x < x + width
        and y <= center_y < y + height
    )


def _next_output_dir(output_root):
    output_root.mkdir(parents=True, exist_ok=True)
    prefix = "teacher_dataset_run_"
    existing_indices = []
    for path in output_root.iterdir():
        if not path.is_dir() or not path.name.startswith(prefix):
            continue
        suffix = path.name[len(prefix) :]
        if suffix.isdigit():
            existing_indices.append(int(suffix))

    index = max(existing_indices, default=0) + 1
    if index >= 10000:
        raise RuntimeError("No free teacher_dataset_run directory")

    candidate = output_root / "{}{:04d}".format(prefix, index)
    candidate.mkdir()
    return candidate


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while True:
            block = source.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _stable_split(text, validation_fraction):
    value = classical._stable_fraction(text)
    return "val" if value < validation_fraction else "train"


def _teacher_boxes(result, confidence_threshold):
    boxes = []
    if result.boxes is None:
        return boxes
    xyxy_values = result.boxes.xyxy.cpu().numpy()
    confidence_values = result.boxes.conf.cpu().numpy()
    for xyxy, confidence in zip(xyxy_values, confidence_values):
        confidence = float(confidence)
        if confidence < confidence_threshold:
            continue
        x1, y1, x2, y2 = (float(value) for value in xyxy)
        width = x2 - x1
        height = y2 - y1
        if width <= 0.0 or height <= 0.0:
            continue
        aspect = width / height
        if not 0.65 <= aspect <= 1.45:
            continue
        boxes.append(
            TeacherBox(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                confidence=confidence,
            )
        )
    return boxes


def _nms(boxes, confidence_threshold, iou_threshold):
    if not boxes:
        return []
    order = sorted(
        range(len(boxes)),
        key=lambda index: boxes[index].confidence,
        reverse=True,
    )
    kept = []
    while order:
        current_index = order.pop(0)
        current = boxes[current_index]
        kept.append(current)
        remaining = []
        current_area = max(0.0, current.x2 - current.x1) * max(
            0.0,
            current.y2 - current.y1,
        )
        for other_index in order:
            other = boxes[other_index]
            intersection_width = max(
                0.0,
                min(current.x2, other.x2) - max(current.x1, other.x1),
            )
            intersection_height = max(
                0.0,
                min(current.y2, other.y2) - max(current.y1, other.y1),
            )
            intersection = intersection_width * intersection_height
            other_area = max(0.0, other.x2 - other.x1) * max(
                0.0,
                other.y2 - other.y1,
            )
            union = current_area + other_area - intersection
            iou = intersection / union if union > 0.0 else 0.0
            if iou <= iou_threshold:
                remaining.append(other_index)
        order = remaining
    return [
        box
        for box in kept
        if box.confidence >= confidence_threshold
    ]


def _onnx_teacher_boxes(
    session,
    input_name,
    image,
    image_size,
    probe_threshold,
    confidence_threshold,
):
    height, width = image.shape[:2]
    scale = min(image_size / float(width), image_size / float(height))
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    pad_left = (image_size - resized_width) // 2
    pad_top = (image_size - resized_height) // 2
    network_image = np.full(
        (image_size, image_size, 3),
        114,
        dtype=np.uint8,
    )
    network_image[
        pad_top : pad_top + resized_height,
        pad_left : pad_left + resized_width,
    ] = resized
    tensor = (
        cv2.cvtColor(network_image, cv2.COLOR_BGR2RGB)
        .transpose(2, 0, 1)
        .astype(np.float32)
        / 255.0
    )
    output = session.run(None, {input_name: tensor[None]})[0]
    predictions = np.asarray(output)
    if predictions.ndim != 3 or predictions.shape[0] != 1:
        raise RuntimeError(
            "Unsupported ONNX output shape: {}".format(predictions.shape)
        )
    predictions = predictions[0]
    if predictions.shape[0] <= predictions.shape[1]:
        predictions = predictions.transpose(1, 0)
    if predictions.shape[1] < 5:
        raise RuntimeError(
            "ONNX output needs at least 5 channels, got {}".format(
                predictions.shape
            )
        )

    boxes = []
    for prediction in predictions:
        confidence = float(np.max(prediction[4:]))
        if confidence < probe_threshold:
            continue
        center_x, center_y, box_width, box_height = (
            float(value) for value in prediction[:4]
        )
        x1 = (center_x - 0.5 * box_width - pad_left) / scale
        y1 = (center_y - 0.5 * box_height - pad_top) / scale
        x2 = (center_x + 0.5 * box_width - pad_left) / scale
        y2 = (center_y + 0.5 * box_height - pad_top) / scale
        x1 = max(0.0, min(float(width - 1), x1))
        y1 = max(0.0, min(float(height - 1), y1))
        x2 = max(0.0, min(float(width - 1), x2))
        y2 = max(0.0, min(float(height - 1), y2))
        box_width = x2 - x1
        box_height = y2 - y1
        if box_width <= 0.0 or box_height <= 0.0:
            continue
        aspect = box_width / box_height
        if not 0.65 <= aspect <= 1.45:
            continue
        boxes.append(
            TeacherBox(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                confidence=confidence,
            )
        )
    return _nms(boxes, confidence_threshold, 0.45)


def _pair_cost(box, candidate):
    distance = math.hypot(
        box.center_x - candidate.center_x,
        box.center_y - candidate.center_y,
    )
    allowed_distance = 0.75 * max(box.radius, candidate.radius)
    if distance > allowed_distance:
        return None
    radius_ratio = box.radius / float(candidate.radius)
    if not 0.55 <= radius_ratio <= 1.80:
        return None
    return distance / max(1.0, allowed_distance)


def _match_one_to_one(boxes, candidates):
    options = []
    for box_index, box in enumerate(boxes):
        for candidate_index, candidate in enumerate(candidates):
            cost = _pair_cost(box, candidate)
            if cost is not None:
                options.append((cost, box_index, candidate_index))
    options.sort()

    pairs = []
    used_boxes = set()
    used_candidates = set()
    for cost, box_index, candidate_index in options:
        if box_index in used_boxes or candidate_index in used_candidates:
            continue
        used_boxes.add(box_index)
        used_candidates.add(candidate_index)
        pairs.append((box_index, candidate_index, cost))
    return (
        pairs,
        [index for index in range(len(boxes)) if index not in used_boxes],
        [
            index
            for index in range(len(candidates))
            if index not in used_candidates
        ],
    )


def _change_coverage(mask, boxes, roi):
    height, width = mask.shape[:2]
    if roi is None:
        roi_x, roi_y, roi_width, roi_height = 0, 0, width, height
    else:
        roi_x, roi_y, roi_width, roi_height = roi
    roi_x2 = min(width, roi_x + roi_width)
    roi_y2 = min(height, roi_y + roi_height)

    selected = np.zeros(mask.shape, dtype=mask.dtype)
    selected[roi_y:roi_y2, roi_x:roi_x2] = mask[
        roi_y:roi_y2,
        roi_x:roi_x2,
    ]
    changed = cv2.countNonZero(selected)
    changed_fraction = changed / float(
        max(1, (roi_x2 - roi_x) * (roi_y2 - roi_y))
    )
    if changed == 0:
        return 1.0, changed_fraction

    covered = np.zeros(mask.shape, dtype=mask.dtype)
    for box in boxes:
        padding = 0.20 * max(
            box.x2 - box.x1,
            box.y2 - box.y1,
        )
        x1 = max(roi_x, int(math.floor(box.x1 - padding)))
        y1 = max(roi_y, int(math.floor(box.y1 - padding)))
        x2 = min(roi_x2 - 1, int(math.ceil(box.x2 + padding)))
        y2 = min(roi_y2 - 1, int(math.ceil(box.y2 + padding)))
        if x1 <= x2 and y1 <= y2:
            cv2.rectangle(covered, (x1, y1), (x2, y2), 255, -1)
    explained = cv2.countNonZero(cv2.bitwise_and(selected, covered))
    return explained / float(changed), changed_fraction


def _yolo_lines(boxes, width, height):
    lines = []
    for box in boxes:
        x1 = max(0.0, min(float(width - 1), box.x1))
        y1 = max(0.0, min(float(height - 1), box.y1))
        x2 = max(0.0, min(float(width - 1), box.x2))
        y2 = max(0.0, min(float(height - 1), box.y2))
        box_width = max(1.0, x2 - x1)
        box_height = max(1.0, y2 - y1)
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)
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


def _draw_preview(
    image,
    status,
    boxes,
    candidates,
    unmatched_boxes,
    unmatched_candidates,
):
    preview = image.copy()
    unmatched_box_set = set(unmatched_boxes)
    unmatched_candidate_set = set(unmatched_candidates)

    for index, candidate in enumerate(candidates):
        color = (
            (0, 165, 255)
            if index in unmatched_candidate_set
            else (255, 200, 0)
        )
        cv2.circle(
            preview,
            (candidate.center_x, candidate.center_y),
            candidate.radius,
            color,
            1,
            cv2.LINE_AA,
        )

    for index, box in enumerate(boxes):
        color = (
            (0, 0, 255)
            if index in unmatched_box_set
            else (0, 255, 0)
        )
        x1, y1, x2, y2 = (
            int(round(box.x1)),
            int(round(box.y1)),
            int(round(box.x2)),
            int(round(box.y2)),
        )
        cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            preview,
            "{:.2f}".format(box.confidence),
            (x1, max(38, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    accepted = status == "accepted_agreement"
    color = (0, 255, 0) if accepted else (0, 0, 255)
    cv2.rectangle(preview, (0, 0), (image.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(
        preview,
        "{} teacher:{} circle:{}".format(
            status,
            len(boxes),
            len(candidates),
        ),
        (6, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        color,
        1,
        cv2.LINE_AA,
    )
    return preview


def _write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _load_exclusions(path):
    if path is None or not path.is_file():
        return set()
    exclusions = set()
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line and not line.startswith("#"):
            exclusions.add(line.replace("\\", "/"))
    return exclusions


def _crop_image_and_boxes(image, boxes, roi):
    if roi is None:
        return image, boxes
    x, y, width, height = roi
    x2 = min(image.shape[1], x + width)
    y2 = min(image.shape[0], y + height)
    cropped = image[y:y2, x:x2]
    shifted = []
    for box in boxes:
        shifted.append(
            TeacherBox(
                x1=max(0.0, box.x1 - x),
                y1=max(0.0, box.y1 - y),
                x2=min(float(cropped.shape[1] - 1), box.x2 - x),
                y2=min(float(cropped.shape[0] - 1), box.y2 - y),
                confidence=box.confidence,
            )
        )
    return cropped, shifted


def _copy_negative(source_path, output_dir, split, stem, roi):
    image_dir = output_dir / "images" / split
    label_dir = output_dir / "labels" / split
    preview_dir = output_dir / "preview" / split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
    if image is None:
        return False
    image, _ = _crop_image_and_boxes(image, [], roi)
    cv2.imwrite(
        str(image_dir / (stem + source_path.suffix.lower())),
        image,
    )
    _write_text(label_dir / (stem + ".txt"), "")
    cv2.imwrite(
        str(preview_dir / (stem + source_path.suffix.lower())),
        image,
    )
    return True


def _arguments():
    project_dir = Path(__file__).resolve().parents[1]
    workspace_dir = project_dir.parents[1]
    default_weights = (
        workspace_dir
        / "contest_library"
        / "01_open_source_projects"
        / "k230_steel_ball_detection_xhz2127"
        / "best.pt"
    )
    parser = argparse.ArgumentParser(
        description="Conservative teacher-assisted steel-ball labelling"
    )
    parser.add_argument(
        "--captures",
        type=Path,
        default=project_dir / "captures",
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=default_weights,
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_dir / "datasets",
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
        help="Only accept object centers inside x,y,width,height",
    )
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--teacher-conf", type=float, default=0.35)
    parser.add_argument("--teacher-probe-conf", type=float, default=0.15)
    parser.add_argument("--circle-score", type=float, default=0.78)
    parser.add_argument("--min-radius", type=int, default=20)
    parser.add_argument("--max-radius", type=int, default=34)
    parser.add_argument(
        "--min-explained-change",
        type=float,
        default=0.0,
        help=(
            "Require this fraction of ROI background change to be "
            "covered by accepted boxes; use only for fixed-camera scenes"
        ),
    )
    parser.add_argument(
        "--max-changed-fraction",
        type=float,
        default=1.0,
        help=(
            "Reject frames whose changed ROI fraction exceeds this value; "
            "useful for excluding hands in fixed-camera scenes"
        ),
    )
    parser.add_argument("--validation-fraction", type=float, default=0.20)
    parser.add_argument(
        "--exclusions",
        type=Path,
        default=(
            project_dir
            / "quality_overrides"
            / "teacher_exclusions.txt"
        ),
    )
    return parser.parse_args()


def main():
    args = _arguments()
    if not args.weights.is_file():
        raise RuntimeError("Teacher weights not found: {}".format(args.weights))

    config_dir = Path(tempfile.gettempdir()) / "diansai_ultralytics"
    config_dir.mkdir(parents=True, exist_ok=True)
    os.environ["YOLO_CONFIG_DIR"] = str(config_dir)

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
        raise RuntimeError("No scene_* directories in {}".format(args.captures))

    output_dir = _next_output_dir(args.output_root)
    exclusions = _load_exclusions(args.exclusions)
    report = {
        "captures": str(args.captures.resolve()),
        "weights": str(args.weights.resolve()),
        "weights_sha256": _sha256(args.weights),
        "output": str(output_dir.resolve()),
        "settings": {
            "imgsz": args.imgsz,
            "teacher_conf": args.teacher_conf,
            "teacher_probe_conf": args.teacher_probe_conf,
            "circle_score": args.circle_score,
            "min_radius": args.min_radius,
            "max_radius": args.max_radius,
            "min_explained_change": args.min_explained_change,
            "max_changed_fraction": args.max_changed_fraction,
            "scene_filter": args.scenes,
            "roi": args.roi,
            "validation_fraction": args.validation_fraction,
            "exclusions": (
                str(args.exclusions.resolve())
                if args.exclusions is not None
                else None
            ),
            "exclusion_count": len(exclusions),
        },
        "counts": {
            "accepted": 0,
            "known_negative": 0,
            "review": 0,
            "boxes": 0,
            "unreadable": 0,
        },
        "status_counts": {},
        "scenes": {},
    }
    rows = []
    onnx_session = None
    onnx_input_name = None
    model = None
    if args.weights.suffix.lower() == ".onnx":
        import onnxruntime as ort

        onnx_session = ort.InferenceSession(
            str(args.weights),
            providers=["CPUExecutionProvider"],
        )
        onnx_input_name = onnx_session.get_inputs()[0].name
    else:
        from ultralytics import YOLO

        model = YOLO(str(args.weights))

    classical.MAX_CHANGED_FRAME_FRACTION = 1.0
    classical.MAX_UNEXPLAINED_CHANGE = 1.0

    for scene_dir in scene_dirs:
        background, background_paths = classical._load_background(scene_dir)
        sample_paths = _image_paths(scene_dir / "samples")
        report["scenes"][scene_dir.name] = {
            "samples": len(sample_paths),
            "background_images": len(background_paths),
        }

        for index, background_path in enumerate(background_paths):
            if index % 5:
                continue
            split = _stable_split(
                "{}:negative:{}".format(scene_dir.name, index),
                args.validation_fraction,
            )
            stem = "{}__negative_{:04d}".format(scene_dir.name, index)
            if _copy_negative(
                background_path,
                output_dir,
                split,
                stem,
                args.roi,
            ):
                report["counts"]["known_negative"] += 1

        results = None
        if model is not None:
            results = model.predict(
                source=[str(path) for path in sample_paths],
                imgsz=args.imgsz,
                conf=args.teacher_probe_conf,
                iou=0.45,
                device="cpu",
                verbose=False,
            )

        for sample_index, sample_path in enumerate(sample_paths):
            image = cv2.imread(str(sample_path), cv2.IMREAD_COLOR)
            if image is None or image.shape != background.shape:
                report["counts"]["unreadable"] += 1
                continue

            if onnx_session is not None:
                detected_boxes = _onnx_teacher_boxes(
                    onnx_session,
                    onnx_input_name,
                    image,
                    args.imgsz,
                    args.teacher_probe_conf,
                    args.teacher_conf,
                )
            else:
                detected_boxes = _teacher_boxes(
                    results[sample_index],
                    args.teacher_conf,
                )
            boxes = [
                box
                for box in detected_boxes
                if _inside_roi(box.center_x, box.center_y, args.roi)
            ]
            classical_result = classical._annotate_frame(
                image,
                background,
                args.min_radius,
                args.max_radius,
                0.58,
            )
            candidates = [
                candidate
                for candidate in classical_result["all_candidates"]
                if (
                    candidate.score >= args.circle_score
                    and _inside_roi(
                        candidate.center_x,
                        candidate.center_y,
                        args.roi,
                    )
                )
            ]
            pairs, unmatched_boxes, unmatched_candidates = (
                _match_one_to_one(boxes, candidates)
            )
            explained_change, changed_fraction = _change_coverage(
                classical_result["mask"],
                boxes,
                args.roi,
            )

            frame_key = "{}/{}".format(
                scene_dir.name,
                sample_path.name,
            )
            if frame_key in exclusions:
                status = "review_manual_exclusion"
            elif not boxes:
                status = "review_no_teacher"
            elif unmatched_boxes:
                status = "review_unmatched_teacher"
            elif unmatched_candidates:
                status = "review_unmatched_circle"
            elif changed_fraction > args.max_changed_fraction:
                status = "review_large_change"
            elif explained_change < args.min_explained_change:
                status = "review_unexplained_change"
            else:
                status = "accepted_agreement"

            report["status_counts"][status] = (
                report["status_counts"].get(status, 0) + 1
            )
            stem = "{}__{:06d}".format(scene_dir.name, sample_index)
            preview = _draw_preview(
                image,
                status,
                boxes,
                candidates,
                unmatched_boxes,
                unmatched_candidates,
            )

            if status == "accepted_agreement":
                split = _stable_split(
                    "{}:{}".format(scene_dir.name, sample_path.name),
                    args.validation_fraction,
                )
                image_dir = output_dir / "images" / split
                label_dir = output_dir / "labels" / split
                preview_dir = output_dir / "preview" / split
                image_dir.mkdir(parents=True, exist_ok=True)
                label_dir.mkdir(parents=True, exist_ok=True)
                preview_dir.mkdir(parents=True, exist_ok=True)
                output_image, output_boxes = _crop_image_and_boxes(
                    image,
                    boxes,
                    args.roi,
                )
                cv2.imwrite(
                    str(
                        image_dir
                        / (stem + sample_path.suffix.lower())
                    ),
                    output_image,
                )
                _write_text(
                    label_dir / (stem + ".txt"),
                    "\n".join(
                        _yolo_lines(
                            output_boxes,
                            output_image.shape[1],
                            output_image.shape[0],
                        )
                    )
                    + "\n",
                )
                cv2.imwrite(str(preview_dir / (stem + ".jpg")), preview)
                report["counts"]["accepted"] += 1
                report["counts"]["boxes"] += len(boxes)
            else:
                review_dir = output_dir / "review" / status
                review_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(review_dir / (stem + ".jpg")), preview)
                report["counts"]["review"] += 1

            for box_index, box in enumerate(boxes):
                rows.append(
                    {
                        "scene": scene_dir.name,
                        "image": sample_path.name,
                        "status": status,
                        "box": box_index,
                        "confidence": "{:.4f}".format(box.confidence),
                        "center_x": "{:.2f}".format(box.center_x),
                        "center_y": "{:.2f}".format(box.center_y),
                        "radius": "{:.2f}".format(box.radius),
                        "matched": int(box_index not in unmatched_boxes),
                    }
                )

    _write_text(
        output_dir / "data.yaml",
        (
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            "  0: {}\n"
        ).format(CLASS_NAME),
    )
    _write_text(
        output_dir / "report.json",
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    )
    with (output_dir / "annotations.csv").open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as csv_file:
        fieldnames = [
            "scene",
            "image",
            "status",
            "box",
            "confidence",
            "center_x",
            "center_y",
            "radius",
            "matched",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("Teacher dataset created: {}".format(output_dir))
    print(json.dumps(report["counts"], ensure_ascii=False, indent=2))
    print(json.dumps(report["status_counts"], ensure_ascii=False, indent=2))
    print(
        "WARNING: use more scenes before treating validation metrics as "
        "independent."
    )


if __name__ == "__main__":
    main()
