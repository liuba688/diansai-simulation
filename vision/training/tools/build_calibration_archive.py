"""Build a balanced image-only ZIP for MaixCAM INT8 calibration."""

import argparse
import zipfile
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _images(directory):
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def _even_sample(paths, count):
    if len(paths) < count:
        raise RuntimeError(
            "Need {} images but only found {}".format(count, len(paths))
        )
    return [paths[index * len(paths) // count] for index in range(count)]


def _scene_balanced_sample(paths, count):
    if len(paths) < count:
        raise RuntimeError(
            "Need {} images but only found {}".format(count, len(paths))
        )
    groups = {}
    for path in paths:
        scene_name = path.stem.split("__", 1)[0]
        groups.setdefault(scene_name, []).append(path)

    quotas = {scene_name: 0 for scene_name in groups}
    remaining = count
    while remaining:
        progressed = False
        for scene_name in sorted(groups):
            if quotas[scene_name] >= len(groups[scene_name]):
                continue
            quotas[scene_name] += 1
            remaining -= 1
            progressed = True
            if remaining == 0:
                break
        if not progressed:
            raise RuntimeError("Unable to allocate calibration images")

    selected = []
    for scene_name in sorted(groups):
        selected.extend(
            _even_sample(groups[scene_name], quotas[scene_name])
        )
    return selected


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--train-count", type=int, default=50)
    parser.add_argument("--val-count", type=int, default=50)
    parser.add_argument(
        "--scene-balanced",
        action="store_true",
        help=(
            "Balance the total requested images across scene prefixes "
            "from both train and val"
        ),
    )
    return parser.parse_args()


def main():
    args = _arguments()
    if args.output.exists():
        raise RuntimeError("Output already exists: {}".format(args.output))
    if args.train_count <= 0 or args.val_count <= 0:
        raise ValueError("Image counts must be positive")

    train_images = _images(args.dataset / "images" / "train")
    val_images = _images(args.dataset / "images" / "val")
    if args.scene_balanced:
        selected = _scene_balanced_sample(
            train_images + val_images,
            args.train_count + args.val_count,
        )
    else:
        selected = _even_sample(train_images, args.train_count)
        selected += _even_sample(val_images, args.val_count)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(
        args.output,
        mode="x",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=6,
    ) as archive:
        for image_path in selected:
            archive.write(
                image_path,
                arcname="images/{}".format(image_path.name),
            )

    with zipfile.ZipFile(args.output, mode="r") as archive:
        bad_file = archive.testzip()
        entry_count = len(archive.infolist())
    if bad_file is not None:
        raise RuntimeError("Corrupt ZIP entry: {}".format(bad_file))
    print(
        "Calibration archive created: {} ({} images)".format(
            args.output,
            entry_count,
        )
    )


if __name__ == "__main__":
    main()
