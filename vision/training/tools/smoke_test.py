"""Synthetic smoke test for the automatic annotation pipeline."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from build_dataset import _annotate_frame


WIDTH = 640
HEIGHT = 480
RADIUS = 26


def _background():
    x_gradient = np.linspace(0, 36, WIDTH, dtype=np.float32)
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.float32)
    image[:, :, 0] = 68 + x_gradient
    image[:, :, 1] = 110 + x_gradient
    image[:, :, 2] = 158 + x_gradient
    for x in range(18, WIDTH, 46):
        cv2.line(image, (x, 0), (x + 80, HEIGHT), (48, 80, 120), 2)
    return np.clip(image, 0, 255).astype(np.uint8)


def _draw_ball(image, center_x, center_y, radius=RADIUS):
    cv2.ellipse(
        image,
        (center_x + 7, center_y + radius - 1),
        (radius, max(4, radius // 3)),
        0,
        0,
        360,
        (28, 34, 38),
        -1,
    )
    for y in range(center_y - radius, center_y + radius + 1):
        for x in range(center_x - radius, center_x + radius + 1):
            dx = x - center_x
            dy = y - center_y
            distance = math.hypot(dx, dy)
            if distance > radius:
                continue

            sphere_z = math.sqrt(max(0.0, radius * radius - distance**2))
            light = (
                72.0
                + 3.0 * sphere_z
                - 1.25 * dy
                - 0.65 * dx
            )
            highlight_distance = math.hypot(
                x - (center_x - radius * 0.30),
                y - (center_y - radius * 0.36),
            )
            if highlight_distance < radius * 0.22:
                light += 105.0
            if dy > radius * 0.35:
                light -= 48.0

            value = int(max(12, min(248, light)))
            image[y, x] = (
                min(255, value + 8),
                min(255, value + 5),
                value,
            )
    cv2.circle(
        image,
        (center_x, center_y),
        radius,
        (185, 190, 195),
        2,
        cv2.LINE_AA,
    )


def _write_fixture(root):
    cases = [
        [],
        [(210, 240)],
        [(160, 220), (320, 245), (485, 210)],
        [(220, 245), (270, 245), (320, 245), (370, 245)],
    ]
    for scene_index in range(1, 4):
        scene_dir = root / "scene_{:04d}".format(scene_index)
        background_dir = scene_dir / "background"
        samples_dir = scene_dir / "samples"
        background_dir.mkdir(parents=True, exist_ok=False)
        samples_dir.mkdir(parents=True, exist_ok=False)
        background = _background()
        for index in range(5):
            cv2.imwrite(
                str(background_dir / "background_{:05d}.jpg".format(index)),
                background,
            )
        for index, centers in enumerate(cases):
            image = background.copy()
            for center_x, center_y in centers:
                _draw_ball(image, center_x, center_y)
            cv2.imwrite(
                str(samples_dir / "sample_{:05d}.jpg".format(index)),
                image,
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-dir", type=Path)
    args = parser.parse_args()

    background = _background()
    cases = [
        ("empty", [], 0),
        ("single", [(210, 240)], 1),
        ("separated", [(160, 220), (320, 245), (485, 210)], 3),
        (
            "touching",
            [(220, 245), (270, 245), (320, 245), (370, 245)],
            4,
        ),
    ]
    failures = []
    for name, centers, expected in cases:
        image = background.copy()
        for center_x, center_y in centers:
            _draw_ball(image, center_x, center_y)

        result = _annotate_frame(
            image,
            background,
            min_radius=20,
            max_radius=34,
            accept_score=0.58,
        )
        actual = len(result["detections"])
        print(
            "{}: expected={} actual={} status={}".format(
                name,
                expected,
                actual,
                result["status"],
            )
        )
        if actual != expected:
            failures.append((name, expected, actual, result["status"]))

    if failures:
        raise AssertionError("Smoke-test failures: {}".format(failures))
    print("automatic annotation smoke test passed")
    if args.fixture_dir is not None:
        _write_fixture(args.fixture_dir)
        print("fixture created: {}".format(args.fixture_dir))


if __name__ == "__main__":
    main()
