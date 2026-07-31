"""Capture background-independent steel-ball dataset scenes on MaixCAM.

Each run creates one scene. Keep the scene empty during the background phase,
then place or move steel balls during the sample phase. The PC-side builder
uses the empty reference frames to create background-independent annotations.
"""

import json
import os
import time

from maix import app, camera, display, image


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
VERTICAL_CENTER_X = FRAME_WIDTH // 2
OUTPUT_ROOT = "/root/steel_ball_captures"

BACKGROUND_CAPTURE_SECONDS = 3.0
PLACEMENT_DELAY_SECONDS = 6.0
SAMPLE_CAPTURE_SECONDS = 90.0

BACKGROUND_INTERVAL_SECONDS = 0.20
SAMPLE_INTERVAL_SECONDS = 0.20
JPEG_QUALITY = 92

_clock = getattr(time, "monotonic", time.time)


def _next_scene_paths():
    os.makedirs(OUTPUT_ROOT, exist_ok=True)
    for scene_index in range(1, 10000):
        scene_name = "scene_{:04d}".format(scene_index)
        scene_dir = os.path.join(OUTPUT_ROOT, scene_name)
        if os.path.exists(scene_dir):
            continue

        background_dir = os.path.join(scene_dir, "background")
        samples_dir = os.path.join(scene_dir, "samples")
        os.makedirs(background_dir, exist_ok=False)
        os.makedirs(samples_dir, exist_ok=False)
        return scene_name, scene_dir, background_dir, samples_dir

    raise RuntimeError("No free scene directory under {}".format(OUTPUT_ROOT))


def _save_frame(frame, directory, prefix, index):
    filename = "{}_{:05d}.jpg".format(prefix, index)
    path = os.path.join(directory, filename)
    frame.save(path, quality=JPEG_QUALITY)
    return path


def _draw_status(frame, title, detail, color):
    frame.draw_rect(0, 0, FRAME_WIDTH, 58, image.COLOR_BLACK, thickness=-1)
    frame.draw_string(8, 6, title, color=color, scale=1.2)
    frame.draw_string(8, 32, detail, color=image.COLOR_WHITE, scale=0.9)
    frame.draw_line(
        VERTICAL_CENTER_X,
        0,
        VERTICAL_CENTER_X,
        FRAME_HEIGHT - 1,
        image.COLOR_GREEN,
        2,
    )


def main():
    scene_name, scene_dir, background_dir, samples_dir = (
        _next_scene_paths()
    )
    metadata = {
        "scene": scene_name,
        "width": FRAME_WIDTH,
        "height": FRAME_HEIGHT,
        "background_seconds": BACKGROUND_CAPTURE_SECONDS,
        "placement_delay_seconds": PLACEMENT_DELAY_SECONDS,
        "sample_seconds": SAMPLE_CAPTURE_SECONDS,
        "sample_interval_seconds": SAMPLE_INTERVAL_SECONDS,
        "instructions": (
            "Background phase must contain no steel balls. Camera and "
            "background must remain fixed during the sample phase."
        ),
    }
    with open(
        os.path.join(scene_dir, "meta.json"),
        "w",
        encoding="utf-8",
    ) as metadata_file:
        json.dump(metadata, metadata_file, ensure_ascii=False, indent=2)

    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
    disp = display.Display()
    background_count = 0
    sample_count = 0
    start_time = _clock()
    next_background_time = 0.0
    next_sample_time = 0.0
    finished = False

    print("Dataset capture started: {}".format(scene_name))
    print("Output: {}".format(scene_dir))
    print("Keep the view EMPTY during the first background phase")

    try:
        cam.skip_frames(20)
        while not app.need_exit():
            frame = cam.read()
            now = _clock()
            elapsed = now - start_time

            background_end = BACKGROUND_CAPTURE_SECONDS
            placement_end = background_end + PLACEMENT_DELAY_SECONDS
            sample_end = placement_end + SAMPLE_CAPTURE_SECONDS

            if elapsed < background_end:
                if elapsed >= next_background_time:
                    _save_frame(
                        frame,
                        background_dir,
                        "background",
                        background_count,
                    )
                    background_count += 1
                    next_background_time += BACKGROUND_INTERVAL_SECONDS

                remaining = max(0.0, background_end - elapsed)
                _draw_status(
                    frame,
                    "EMPTY BACKGROUND",
                    "Do not place balls  {:.1f}s".format(remaining),
                    image.COLOR_YELLOW,
                )
            elif elapsed < placement_end:
                remaining = max(0.0, placement_end - elapsed)
                _draw_status(
                    frame,
                    "PLACE / MOVE BALLS",
                    "Recording starts in {:.1f}s".format(remaining),
                    image.COLOR_RED,
                )
            elif elapsed < sample_end:
                sample_elapsed = elapsed - placement_end
                if sample_elapsed >= next_sample_time:
                    _save_frame(
                        frame,
                        samples_dir,
                        "sample",
                        sample_count,
                    )
                    sample_count += 1
                    next_sample_time += SAMPLE_INTERVAL_SECONDS

                remaining = max(0.0, sample_end - elapsed)
                _draw_status(
                    frame,
                    "RECORDING {}".format(sample_count),
                    "Change count/position  {:.1f}s".format(remaining),
                    image.COLOR_GREEN,
                )
            else:
                if not finished:
                    metadata["background_images"] = background_count
                    metadata["sample_images"] = sample_count
                    with open(
                        os.path.join(scene_dir, "meta.json"),
                        "w",
                        encoding="utf-8",
                    ) as metadata_file:
                        json.dump(
                            metadata,
                            metadata_file,
                            ensure_ascii=False,
                            indent=2,
                        )
                    print(
                        "Capture complete: {} background, {} samples".format(
                            background_count,
                            sample_count,
                        )
                    )
                    finished = True

                _draw_status(
                    frame,
                    "CAPTURE COMPLETE",
                    "{}  samples:{}".format(scene_name, sample_count),
                    image.COLOR_GREEN,
                )

            disp.show(frame)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
