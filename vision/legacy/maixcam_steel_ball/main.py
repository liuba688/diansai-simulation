"""MaixCAM real-time steel-ball detector using the trained YOLOv8 model."""

import os
import time

from maix import app, camera, display, image, nn


MODEL_FILE_NAME = "steel_ball_320_v2.mud"
MODEL_BINARY_FILE_NAME = "steel_ball_320_v2.cvimodel"
_BUNDLED_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    MODEL_FILE_NAME,
)
_PERSISTENT_MODEL_PATH = (
    "/root/models/steel_ball_320_v2/steel_ball_320_v2.mud"
)
MODEL_PATH = (
    _BUNDLED_MODEL_PATH
    if os.path.exists(_BUNDLED_MODEL_PATH)
    else _PERSISTENT_MODEL_PATH
)
MODEL_BINARY_PATH = os.path.join(
    os.path.dirname(MODEL_PATH),
    MODEL_BINARY_FILE_NAME,
)
DETECTOR_CONFIDENCE_THRESHOLD = 0.45
CANDIDATE_CONFIDENCE_THRESHOLD = 0.45
DISPLAY_CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.35
MIN_ASPECT_RATIO = 0.70
MAX_ASPECT_RATIO = 1.0 / MIN_ASPECT_RATIO
MIN_BOX_SIDE = 8
TRACK_CONFIRM_FRAMES = 3
TRACK_MAX_MISSES = 2
TRACK_MATCH_DISTANCE_FACTOR = 1.25
TRACK_SMOOTHING = 0.60
LOG_INTERVAL_SECONDS = 0.5

_clock = getattr(time, "monotonic", time.time)


def _require_model():
    missing_paths = [
        path
        for path in (MODEL_PATH, MODEL_BINARY_PATH)
        if not os.path.exists(path)
    ]
    if missing_paths:
        raise RuntimeError(
            "Model file missing: {}. Keep the matching .mud and "
            ".cvimodel in the same directory.".format(
                ", ".join(missing_paths)
            )
        )


def _candidate_from_detection(detection):
    score = float(detection.score)
    width = int(detection.w)
    height = int(detection.h)
    if score < CANDIDATE_CONFIDENCE_THRESHOLD:
        return None
    if width < MIN_BOX_SIDE or height < MIN_BOX_SIDE:
        return None

    aspect_ratio = width / float(height)
    if not MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO:
        return None

    x = int(detection.x)
    y = int(detection.y)
    return {
        "x": x,
        "y": y,
        "w": width,
        "h": height,
        "cx": x + width * 0.5,
        "cy": y + height * 0.5,
        "score": score,
    }


def _update_tracks(tracks, candidates):
    unmatched_tracks = set(range(len(tracks)))
    matched_candidates = set()
    possible_matches = []

    for track_index, track in enumerate(tracks):
        for candidate_index, candidate in enumerate(candidates):
            track = tracks[track_index]
            size_ratio = max(
                candidate["w"] / float(max(1, track["w"])),
                track["w"] / float(max(1, candidate["w"])),
                candidate["h"] / float(max(1, track["h"])),
                track["h"] / float(max(1, candidate["h"])),
            )
            if size_ratio > 1.8:
                continue

            dx = candidate["cx"] - track["cx"]
            dy = candidate["cy"] - track["cy"]
            distance_squared = dx * dx + dy * dy
            match_distance = max(
                10.0,
                TRACK_MATCH_DISTANCE_FACTOR
                * max(
                    candidate["w"],
                    candidate["h"],
                    track["w"],
                    track["h"],
                ),
            )
            if distance_squared > match_distance * match_distance:
                continue
            possible_matches.append(
                (distance_squared, track_index, candidate_index)
            )

    for _, track_index, candidate_index in sorted(possible_matches):
        if track_index not in unmatched_tracks:
            continue
        if candidate_index in matched_candidates:
            continue

        track = tracks[track_index]
        candidate = candidates[candidate_index]
        keep = 1.0 - TRACK_SMOOTHING
        for key in ("x", "y", "w", "h", "cx", "cy", "score"):
            track[key] = (
                keep * track[key]
                + TRACK_SMOOTHING * candidate[key]
            )
        track["hits"] += 1
        track["misses"] = 0
        unmatched_tracks.remove(track_index)
        matched_candidates.add(candidate_index)

    for track_index in unmatched_tracks:
        tracks[track_index]["misses"] += 1
        tracks[track_index]["hits"] = max(
            0,
            tracks[track_index]["hits"] - 1,
        )

    for candidate_index, candidate in enumerate(candidates):
        if candidate_index in matched_candidates:
            continue
        candidate["hits"] = 1
        candidate["misses"] = 0
        tracks.append(candidate)

    tracks[:] = [
        track
        for track in tracks
        if track["misses"] <= TRACK_MAX_MISSES
    ]
    return [
        track
        for track in tracks
        if (
            track["misses"] == 0
            and track["hits"] >= TRACK_CONFIRM_FRAMES
            and track["score"] >= DISPLAY_CONFIDENCE_THRESHOLD
        )
    ]


def _draw_track(frame, track, index):
    score = float(track["score"])
    x = int(track["x"])
    y = int(track["y"])
    width = int(track["w"])
    height = int(track["h"])

    frame.draw_rect(
        x,
        y,
        width,
        height,
        color=image.COLOR_RED,
        thickness=2,
    )
    frame.draw_string(
        x,
        max(0, y - 18),
        "BALL{} {:.0f}%".format(index + 1, score * 100.0),
        color=image.COLOR_RED,
        scale=1.0,
    )


def main():
    _require_model()
    detector = nn.YOLOv8(model=MODEL_PATH, dual_buff=True)
    cam = camera.Camera(
        detector.input_width(),
        detector.input_height(),
        detector.input_format(),
    )
    disp = display.Display()

    frame_count = 0
    fps = 0.0
    previous_time = _clock()
    last_log_time = 0.0
    tracks = []

    print("Steel-ball YOLOv8 detector started")
    print("model={}".format(MODEL_PATH))
    print("model_binary={}".format(MODEL_BINARY_PATH))
    print(
        "input={}x{} detect_conf={:.2f} display_conf={:.2f} "
        "iou={:.2f}".format(
            detector.input_width(),
            detector.input_height(),
            DETECTOR_CONFIDENCE_THRESHOLD,
            DISPLAY_CONFIDENCE_THRESHOLD,
            IOU_THRESHOLD,
        )
    )

    try:
        cam.skip_frames(10)
        while not app.need_exit():
            frame = cam.read()
            detections = detector.detect(
                frame,
                conf_th=DETECTOR_CONFIDENCE_THRESHOLD,
                iou_th=IOU_THRESHOLD,
            )
            candidates = []
            for detection in detections:
                candidate = _candidate_from_detection(detection)
                if candidate is not None:
                    candidates.append(candidate)
            confirmed_tracks = _update_tracks(tracks, candidates)

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

            for index, track in enumerate(confirmed_tracks):
                _draw_track(frame, track, index)

            frame.draw_string(
                6,
                6,
                "balls:{} fps:{:.1f}".format(
                    len(confirmed_tracks),
                    fps,
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )

            if now - last_log_time >= LOG_INTERVAL_SECONDS:
                summary = ", ".join(
                    "x={} y={} w={} h={} conf={:.0f}".format(
                        int(item["x"]),
                        int(item["y"]),
                        int(item["w"]),
                        int(item["h"]),
                        float(item["score"]) * 100.0,
                    )
                    for item in confirmed_tracks
                )
                print(
                    "balls={} candidates={} raw={} fps={:.1f} {}".format(
                        len(confirmed_tracks),
                        len(candidates),
                        len(detections),
                        fps,
                        summary,
                    )
                )
                last_log_time = now

            disp.show(frame)
            frame_count += 1
    finally:
        cam.close()


if __name__ == "__main__":
    main()
