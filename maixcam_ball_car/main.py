"""Steel-ball detection and UART guidance for the MSPM0 line-following car."""

import os
import time

from maix import app, camera, display, image, nn

import config
from ball_tracker import BallTracker
from vision_protocol import (
    TARGET_FLAG_CLOSE,
    TARGET_FLAG_CONFIRMED,
    TARGET_FLAG_MULTIPLE,
    TARGET_FLAG_VALID,
)
from vision_uart import VisionUartLink


APP_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(APP_DIRECTORY, config.MODEL_FILE_NAME)
MODEL_BINARY_PATH = os.path.join(
    APP_DIRECTORY,
    config.MODEL_BINARY_FILE_NAME,
)
_clock = getattr(time, "monotonic", time.time)

CAR_STATE_NAMES = (
    "IDLE",
    "LINE",
    "CONFIRM",
    "LOCK",
    "APPROACH",
    "CREEP",
    "PICKUP",
    "BACKTRACK",
    "REACQUIRE",
    "CARRY",
    "FAULT",
)


def _require_model():
    missing = [
        path
        for path in (MODEL_PATH, MODEL_BINARY_PATH)
        if not os.path.exists(path)
    ]
    if missing:
        raise RuntimeError(
            "Model file missing: {}. Keep the matching .mud and "
            ".cvimodel beside main.py.".format(", ".join(missing))
        )


def _track_box(track):
    return (
        int(track["x"]),
        int(track["y"]),
        max(1, int(track["w"])),
        max(1, int(track["h"])),
    )


def _draw_track(frame, track, selected):
    x, y, width, height = _track_box(track)
    color = image.COLOR_GREEN if selected else image.COLOR_RED
    prefix = "TARGET" if selected else "BALL"
    frame.draw_rect(
        x,
        y,
        width,
        height,
        color=color,
        thickness=3 if selected else 2,
    )
    frame.draw_string(
        x,
        max(0, y - 18),
        "{} {:.0f}%".format(prefix, track["score"] * 100.0),
        color=color,
        scale=1.0,
    )


def _target_values(selected, controllable_count, frame_width, frame_height):
    if selected is None:
        return {
            "flags": 0,
            "center_x": 0,
            "center_y": 0,
            "width": 0,
            "height": 0,
            "confidence": 0,
            "count": controllable_count,
        }

    flags = TARGET_FLAG_VALID | TARGET_FLAG_CONFIRMED
    if selected["h"] >= frame_height * config.CLOSE_BOX_HEIGHT_RATIO:
        flags |= TARGET_FLAG_CLOSE
    if controllable_count > 1:
        flags |= TARGET_FLAG_MULTIPLE

    return {
        "flags": flags,
        "center_x": int(selected["cx"]),
        "center_y": int(selected["cy"]),
        "width": int(selected["w"]),
        "height": int(selected["h"]),
        "confidence": selected["score"] * 100.0,
        "count": controllable_count,
    }


def _status_name(status):
    if status is None:
        return "--"
    state = status["state"]
    if 0 <= state < len(CAR_STATE_NAMES):
        return CAR_STATE_NAMES[state]
    return str(state)


def main():
    _require_model()
    detector = nn.YOLOv8(model=MODEL_PATH, dual_buff=True)
    frame_width = detector.input_width()
    frame_height = detector.input_height()
    cam = camera.Camera(
        frame_width,
        frame_height,
        detector.input_format(),
    )
    disp = display.Display()
    tracker = BallTracker(frame_width, frame_height)
    link = VisionUartLink()

    frame_count = 0
    fps = 0.0
    previous_time = _clock()
    last_log_time = 0.0
    next_uart_send_time = 0.0
    next_status_poll_time = 0.0
    car_status = None

    print("MaixCAM ball-car vision started")
    print("model={}".format(MODEL_PATH))
    print(
        "input={}x{} detect={:.2f} display={:.2f} "
        "control={:.2f} iou={:.2f}".format(
            frame_width,
            frame_height,
            config.DETECTOR_CONFIDENCE_THRESHOLD,
            config.DISPLAY_CONFIDENCE_THRESHOLD,
            config.CONTROL_CONFIDENCE_THRESHOLD,
            config.IOU_THRESHOLD,
        )
    )

    try:
        cam.skip_frames(10)
        while not app.need_exit():
            frame = cam.read()
            detections = detector.detect(
                frame,
                conf_th=config.DETECTOR_CONFIDENCE_THRESHOLD,
                iou_th=config.IOU_THRESHOLD,
            )
            candidates = []
            for detection in detections:
                candidate = tracker.candidate_from_detection(detection)
                if candidate is not None:
                    candidates.append(candidate)

            displayed, controllable, selected = tracker.update(candidates)
            target = _target_values(
                selected,
                len(controllable),
                frame_width,
                frame_height,
            )

            now = _clock()
            elapsed = now - previous_time
            previous_time = now
            if elapsed > 1e-6:
                instant_fps = 1.0 / elapsed
                fps = (
                    instant_fps
                    if frame_count == 0
                    else 0.90 * fps + 0.10 * instant_fps
                )

            if now >= next_uart_send_time:
                link.send_target(
                    target["flags"],
                    target["center_x"],
                    target["center_y"],
                    target["width"],
                    target["height"],
                    frame_width,
                    frame_height,
                    target["confidence"],
                    target["count"],
                )
                next_uart_send_time = (
                    now + config.UART_SEND_INTERVAL_SECONDS
                )

            if now >= next_status_poll_time:
                car_status = link.poll_status()
                next_status_poll_time = (
                    now + config.UART_STATUS_POLL_INTERVAL_SECONDS
                )

            selected_id = None if selected is None else selected["id"]
            for track in displayed:
                _draw_track(
                    frame,
                    track,
                    track["id"] == selected_id,
                )

            frame.draw_rect(
                frame_width // 2 - 3,
                frame_height // 2 - 3,
                6,
                6,
                color=image.COLOR_GREEN,
                thickness=1,
            )
            frame.draw_string(
                6,
                6,
                "balls:{} ctrl:{} fps:{:.1f}".format(
                    len(displayed),
                    len(controllable),
                    fps,
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )
            frame.draw_string(
                6,
                24,
                "UART:{} CAR:{}".format(
                    "ON" if link.ready else "OFF",
                    _status_name(car_status),
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )

            if now - last_log_time >= config.LOG_INTERVAL_SECONDS:
                if selected is None:
                    selected_summary = "target=none"
                else:
                    selected_summary = (
                        "target=id{} x={} y={} w={} h={} conf={:.0f} "
                        "close={}"
                    ).format(
                        selected["id"],
                        target["center_x"],
                        target["center_y"],
                        target["width"],
                        target["height"],
                        target["confidence"],
                        bool(target["flags"] & TARGET_FLAG_CLOSE),
                    )
                print(
                    "display={} control={} candidates={} raw={} "
                    "fps={:.1f} uart={} car={} {}".format(
                        len(displayed),
                        len(controllable),
                        len(candidates),
                        len(detections),
                        fps,
                        link.ready,
                        _status_name(car_status),
                        selected_summary,
                    )
                )
                last_log_time = now

            disp.show(frame)
            frame_count += 1
    finally:
        link.close()
        cam.close()


if __name__ == "__main__":
    main()
