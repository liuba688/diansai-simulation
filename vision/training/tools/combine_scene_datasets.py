"""Combine reviewed YOLO datasets while keeping whole scenes in one split."""

import argparse
import json
import shutil
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _next_output_dir(output_root):
    output_root.mkdir(parents=True, exist_ok=True)
    prefix = "combined_dataset_run_"
    indices = []
    for path in output_root.iterdir():
        if not path.is_dir() or not path.name.startswith(prefix):
            continue
        suffix = path.name[len(prefix) :]
        if suffix.isdigit():
            indices.append(int(suffix))
    index = max(indices, default=0) + 1
    if index >= 10000:
        raise RuntimeError("No free combined_dataset_run directory")
    output_dir = output_root / "{}{:04d}".format(prefix, index)
    output_dir.mkdir()
    return output_dir


def _dataset_pairs(dataset_dir):
    pairs = []
    for original_split in ("train", "val"):
        image_dir = dataset_dir / "images" / original_split
        label_dir = dataset_dir / "labels" / original_split
        if not image_dir.is_dir():
            continue
        for image_path in sorted(image_dir.iterdir()):
            if (
                not image_path.is_file()
                or image_path.suffix.lower() not in IMAGE_SUFFIXES
            ):
                continue
            label_path = label_dir / (image_path.stem + ".txt")
            if not label_path.is_file():
                raise RuntimeError(
                    "Missing label for {}".format(image_path)
                )
            pairs.append((image_path, label_path))
    return pairs


def _validate_label(label_path):
    boxes = 0
    for line_number, raw_line in enumerate(
        label_path.read_text(encoding="utf-8-sig").splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise RuntimeError(
                "Invalid YOLO row {}:{}: {}".format(
                    label_path,
                    line_number,
                    raw_line,
                )
            )
        class_id = int(fields[0])
        values = [float(item) for item in fields[1:]]
        if class_id != 0:
            raise RuntimeError(
                "Unexpected class ID in {}".format(label_path)
            )
        center_x, center_y, width, height = values
        if (
            not 0.0 <= center_x <= 1.0
            or not 0.0 <= center_y <= 1.0
            or not 0.0 < width <= 1.0
            or not 0.0 < height <= 1.0
        ):
            raise RuntimeError(
                "Out-of-range YOLO box in {}".format(label_path)
            )
        boxes += 1
    return boxes


def _copy_dataset(dataset_dir, split, output_dir, used_names):
    image_output = output_dir / "images" / split
    label_output = output_dir / "labels" / split
    image_output.mkdir(parents=True, exist_ok=True)
    label_output.mkdir(parents=True, exist_ok=True)

    image_count = 0
    box_count = 0
    empty_count = 0
    scene_names = set()
    for image_path, label_path in _dataset_pairs(dataset_dir):
        output_name = image_path.name
        if output_name in used_names:
            raise RuntimeError(
                "Duplicate image name across sources: {}".format(output_name)
            )
        used_names.add(output_name)

        boxes = _validate_label(label_path)
        shutil.copy2(image_path, image_output / output_name)
        shutil.copy2(label_path, label_output / label_path.name)

        scene_names.add(image_path.stem.split("__", 1)[0])
        image_count += 1
        box_count += boxes
        if boxes == 0:
            empty_count += 1

    return {
        "source": str(dataset_dir.resolve()),
        "split": split,
        "scenes": sorted(scene_names),
        "images": image_count,
        "boxes": box_count,
        "empty_images": empty_count,
    }


def _arguments():
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Combine reviewed YOLO datasets with source datasets assigned "
            "entirely to train or val."
        )
    )
    parser.add_argument(
        "--train",
        type=Path,
        action="append",
        default=[],
        help="Reviewed dataset directory assigned wholly to train",
    )
    parser.add_argument(
        "--val",
        type=Path,
        action="append",
        default=[],
        help="Reviewed dataset directory assigned wholly to val",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_dir / "datasets",
    )
    return parser.parse_args()


def main():
    args = _arguments()
    if not args.train or not args.val:
        raise ValueError("At least one --train and one --val are required")

    sources = [(path, "train") for path in args.train]
    sources += [(path, "val") for path in args.val]
    for path, _ in sources:
        if not path.is_dir():
            raise RuntimeError("Dataset directory not found: {}".format(path))

    output_dir = _next_output_dir(args.output_root)
    used_names = set()
    source_reports = []
    for source_path, split in sources:
        source_reports.append(
            _copy_dataset(source_path, split, output_dir, used_names)
        )

    counts = {
        split: {
            "images": sum(
                item["images"]
                for item in source_reports
                if item["split"] == split
            ),
            "boxes": sum(
                item["boxes"]
                for item in source_reports
                if item["split"] == split
            ),
            "empty_images": sum(
                item["empty_images"]
                for item in source_reports
                if item["split"] == split
            ),
        }
        for split in ("train", "val")
    }
    report = {
        "output": str(output_dir.resolve()),
        "split_policy": "whole source scenes; no scene crosses splits",
        "sources": source_reports,
        "counts": counts,
    }
    (output_dir / "data.yaml").write_text(
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: steel_ball\n",
        encoding="utf-8",
    )
    (output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("Combined dataset created: {}".format(output_dir))
    print(json.dumps(counts, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
