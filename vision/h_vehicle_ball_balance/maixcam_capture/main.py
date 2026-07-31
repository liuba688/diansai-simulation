"""Capture full 640x480 pipe-and-ball scenes on MaixCAM.

Saved JPEG files never contain the preview overlay and are not cropped.
The horizontal field is preserved completely. Vertical cropping is performed
later on the PC so that the ROI can be changed without recollecting images.
"""

import json
import os
import time

from maix import app, camera, display, image


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
OUTPUT_ROOT = "/root/h_ball_pipe_captures"

# Preview only. These values are recorded in meta.json but are not applied to
# saved images. Start generously; crop variants can be generated on the PC.
ROI_CENTER_Y = 240
ROI_PREVIEW_HEIGHT = 160
VERTICAL_CENTER_X = FRAME_WIDTH // 2

BACKGROUND_CAPTURE_SECONDS = 3.0
PLACEMENT_DELAY_SECONDS = 6.0
SAMPLE_CAPTURE_SECONDS = 90.0
BACKGROUND_INTERVAL_SECONDS = 0.20
SAMPLE_INTERVAL_SECONDS = 0.10
JPEG_QUALITY = 95

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


def _save_clean_frame(frame, directory, prefix, index):
    path = os.path.join(
        directory,
        "{}_{:05d}.jpg".format(prefix, index),
    )
    # This function is called before any preview graphics are drawn.
    frame.save(path, quality=JPEG_QUALITY)
    return path


def _draw_roi_preview(frame):
    top = max(0, ROI_CENTER_Y - ROI_PREVIEW_HEIGHT // 2)
    bottom = min(
        FRAME_HEIGHT - 1,
        ROI_CENTER_Y + ROI_PREVIEW_HEIGHT // 2,
    )
    frame.draw_line(0, top, FRAME_WIDTH - 1, top, image.COLOR_YELLOW, 2)
    frame.draw_line(
        0,
        bottom,
        FRAME_WIDTH - 1,
        bottom,
        image.COLOR_YELLOW,
        2,
    )
    frame.draw_line(
        0,
        ROI_CENTER_Y,
        FRAME_WIDTH - 1,
        ROI_CENTER_Y,
        image.COLOR_GREEN,
        1,
    )
    frame.draw_line(
        VERTICAL_CENTER_X,
        0,
        VERTICAL_CENTER_X,
        FRAME_HEIGHT - 1,
        image.COLOR_GREEN,
        2,
    )


def _draw_status(frame, title, detail, color):
    _draw_roi_preview(frame)
    frame.draw_rect(0, 0, FRAME_WIDTH, 58, image.COLOR_BLACK, thickness=-1)
    frame.draw_string(8, 6, title, color=color, scale=1.2)
    frame.draw_string(8, 32, detail, color=image.COLOR_WHITE, scale=0.9)


def _write_metadata(path, metadata):
    with open(path, "w", encoding="utf-8") as metadata_file:
        json.dump(metadata, metadata_file, ensure_ascii=False, indent=2)


def main():
    scene_name, scene_dir, background_dir, samples_dir = (
        _next_scene_paths()
    )
    metadata_path = os.path.join(scene_dir, "meta.json")
    metadata = {
        "scene": scene_name,
        "capture_width": FRAME_WIDTH,
        "capture_height": FRAME_HEIGHT,
        "horizontal_crop": "none",
        "saved_images_are_uncropped": True,
        "roi_preview_center_y": ROI_CENTER_Y,
        "roi_preview_height": ROI_PREVIEW_HEIGHT,
        "background_seconds": BACKGROUND_CAPTURE_SECONDS,
        "placement_delay_seconds": PLACEMENT_DELAY_SECONDS,
        "sample_seconds": SAMPLE_CAPTURE_SECONDS,
        "sample_interval_seconds": SAMPLE_INTERVAL_SECONDS,
        "jpeg_quality": JPEG_QUALITY,
        "mounting": (
            "Camera fixed above the pipe; pipe spans the horizontal field."
        ),
    }
    _write_metadata(metadata_path, metadata)

    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
    disp = display.Display()
    background_count = 0
    sample_count = 0
    start_time = _clock()
    next_background_time = 0.0
    next_sample_time = 0.0
    finished = False

    print("H-task pipe dataset capture started: {}".format(scene_name))
    print("Output: {}".format(scene_dir))
    print("Saved images: full 640x480, no overlay, no crop")

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
                    _save_clean_frame(
                        frame,
                        background_dir,
                        "background",
                        background_count,
                    )
                    background_count += 1
                    next_background_time += BACKGROUND_INTERVAL_SECONDS

                _draw_status(
                    frame,
                    "EMPTY PIPE",
                    "No steel ball  {:.1f}s".format(
                        max(0.0, background_end - elapsed)
                    ),
                    image.COLOR_YELLOW,
                )
            elif elapsed < placement_end:
                _draw_status(
                    frame,
                    "PLACE STEEL BALL",
                    "Recording in {:.1f}s".format(
                        max(0.0, placement_end - elapsed)
                    ),
                    image.COLOR_RED,
                )
            elif elapsed < sample_end:
                sample_elapsed = elapsed - placement_end
                if sample_elapsed >= next_sample_time:
                    _save_clean_frame(
                        frame,
                        samples_dir,
                        "sample",
                        sample_count,
                    )
                    sample_count += 1
                    next_sample_time += SAMPLE_INTERVAL_SECONDS

                _draw_status(
                    frame,
                    "RECORDING {}".format(sample_count),
                    "Move/roll ball  {:.1f}s".format(
                        max(0.0, sample_end - elapsed)
                    ),
                    image.COLOR_GREEN,
                )
            else:
                if not finished:
                    metadata["background_images"] = background_count
                    metadata["sample_images"] = sample_count
                    _write_metadata(metadata_path, metadata)
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
