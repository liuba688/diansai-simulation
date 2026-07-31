"""Low-latency steel-ball vision with direct X42S bench control."""

SOURCE_VERSION = "3.1.4-cal435-r3"
PIXEL_CALIBRATION_ONLY = False

import os
import time

from maix import app, camera, display, image, nn

from task1_timed_control import (
    BallTaskController,
    CENTER_0_PX,
    CONTROL_PROFILE_NAME,
    NEGATIVE_50_PX,
    POSITIVE_50_PX,
)
from x42s_uart import X42SMotor


MODEL_FILE_NAME = "pipe_ball_480x96_v1.mud"
MODEL_BINARY_FILE_NAME = "pipe_ball_480x96_v1.cvimodel"
_BUNDLED_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    MODEL_FILE_NAME,
)
_PERSISTENT_MODEL_PATH = (
    "/root/models/pipe_ball_480x96_v1/pipe_ball_480x96_v1.mud"
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
TRACK_CONFIRM_FRAMES = 1
TRACK_MAX_MISSES = 1
TRACK_MATCH_DISTANCE_FACTOR = 3.0
TRACK_SMOOTHING = 1.0
LOG_INTERVAL_SECONDS = 0.5
FRAME_WIDTH = 640
FRAME_HEIGHT = 128
ROI_X = 0
ROI_Y = 0
ROI_WIDTH = 640
ROI_HEIGHT = 128
CAMERA_TARGET_FPS = 60
SYSTEM_LATENCY_SECONDS = 0.000
MAX_PREDICTION_SECONDS = 0.050
VELOCITY_SMOOTHING = 0.65
MAX_ABS_VELOCITY_PX_S = 2500.0

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
            track["misses"] <= TRACK_MAX_MISSES
            and track["hits"] >= TRACK_CONFIRM_FRAMES
            and track["score"] >= DISPLAY_CONFIDENCE_THRESHOLD
        )
    ]


def _clamp(value, low, high):
    return max(low, min(high, value))


def _predict_track(track, state, now):
    has_measurement = track["misses"] == 0
    if has_measurement:
        measurement_x = float(track["cx"])
        if state["valid"]:
            dt = now - state["measurement_time"]
            if 0.003 <= dt <= 0.200:
                measured_velocity = (
                    measurement_x - state["measurement_x"]
                ) / dt
                measured_velocity = _clamp(
                    measured_velocity,
                    -MAX_ABS_VELOCITY_PX_S,
                    MAX_ABS_VELOCITY_PX_S,
                )
                keep = 1.0 - VELOCITY_SMOOTHING
                state["velocity_x"] = (
                    keep * state["velocity_x"]
                    + VELOCITY_SMOOTHING * measured_velocity
                )
            else:
                state["velocity_x"] = 0.0
        else:
            state["velocity_x"] = 0.0
            state["valid"] = True
        state["measurement_x"] = measurement_x
        state["measurement_time"] = now

    if not state["valid"]:
        return track

    measurement_age = max(0.0, now - state["measurement_time"])
    prediction_time = min(
        MAX_PREDICTION_SECONDS,
        SYSTEM_LATENCY_SECONDS + measurement_age,
    )
    predicted_cx = (
        state["measurement_x"]
        + state["velocity_x"] * prediction_time
    )
    predicted_cx = _clamp(
        predicted_cx,
        track["w"] * 0.5,
        ROI_WIDTH - track["w"] * 0.5,
    )

    predicted = dict(track)
    predicted["raw_cx"] = state["measurement_x"]
    predicted["cx"] = predicted_cx
    predicted["x"] = predicted_cx - track["w"] * 0.5
    predicted["velocity_x"] = state["velocity_x"]
    predicted["prediction_ms"] = prediction_time * 1000.0
    return predicted


def _draw_track(frame, track, index):
    score = float(track["score"])
    x = int(track["x"])
    y = ROI_Y + int(track["y"])
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
    detector = nn.YOLOv8(model=MODEL_PATH, dual_buff=False)
    cam = camera.Camera(
        FRAME_WIDTH,
        FRAME_HEIGHT,
        detector.input_format(),
        fps=CAMERA_TARGET_FPS,
        buff_num=1,
    )
    disp = display.Display()

    frame_count = 0
    fps = 0.0
    previous_time = _clock()
    last_log_time = 0.0
    tracks = []
    motion_state = {
        "valid": False,
        "measurement_x": 0.0,
        "measurement_time": 0.0,
        "velocity_x": 0.0,
    }
    controller = BallTaskController(
        pixel_calibration_only=PIXEL_CALIBRATION_ONLY
    )
    control_snapshot = controller.update(None, 0.0, _clock())
    motor = None
    motor_error = ""
    motor_fault_handled = False

    print(
        "Steel-ball direct X42S controller started "
        "version={} profile={}".format(
            SOURCE_VERSION,
            CONTROL_PROFILE_NAME,
        )
    )
    print("model={}".format(MODEL_PATH))
    print("model_binary={}".format(MODEL_BINARY_PATH))
    print(
        "calibration +50={:.0f}px O={:.0f}px -50={:.0f}px".format(
            POSITIVE_50_PX,
            CENTER_0_PX,
            NEGATIVE_50_PX,
        )
    )
    print(
        "low_latency dual_buff=off camera_buffers=1 "
        "prediction_ms={:.0f}".format(
            SYSTEM_LATENCY_SECONDS * 1000.0,
        )
    )
    print(
        "camera={}x{} requested_fps={}".format(
            cam.width(),
            cam.height(),
            CAMERA_TARGET_FPS,
        )
    )
    if cam.width() != FRAME_WIDTH or cam.height() != FRAME_HEIGHT:
        raise RuntimeError(
            "Camera did not accept 640x128: actual={}x{}".format(
                cam.width(),
                cam.height(),
            )
        )
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
        try:
            motor = X42SMotor()
            motor.initialize()
            print(
                "X42S ready: UART1 A19/A18 115200, "
                "boot position anchored as zero"
            )
        except Exception as error:
            motor_error = str(error)
            controller.force_fault("motor init")
            print("X42S init failed: {}".format(motor_error))
            if motor is not None:
                motor.level_and_stop()

        while not app.need_exit():
            frame = cam.read()
            detections = detector.detect(
                frame,
                conf_th=DETECTOR_CONFIDENCE_THRESHOLD,
                iou_th=IOU_THRESHOLD,
                fit=image.Fit.FIT_CONTAIN,
            )
            candidates = []
            for detection in detections:
                candidate = _candidate_from_detection(detection)
                if candidate is not None:
                    candidates.append(candidate)
            if len(candidates) > 1:
                candidates = [
                    max(candidates, key=lambda item: item["score"])
                ]
            confirmed_tracks = _update_tracks(tracks, candidates)

            now = _clock()
            confirmed_tracks.sort(
                key=lambda item: (
                    item["misses"],
                    -float(item["score"]),
                )
            )
            predicted_tracks = [
                _predict_track(track, motion_state, now)
                for track in confirmed_tracks[:1]
            ]
            if not confirmed_tracks and not tracks:
                motion_state["valid"] = False

            measured_pixel_x = None
            measured_confidence = 0.0
            if confirmed_tracks and confirmed_tracks[0]["misses"] == 0:
                measured_pixel_x = float(confirmed_tracks[0]["cx"])
                measured_confidence = float(
                    confirmed_tracks[0]["score"]
                )

            if motor is not None and motor.initialized:
                if not motor.poll(now) and not motor_fault_handled:
                    controller.force_fault("motor response")
                    motor.best_effort_level()
                    motor_fault_handled = True

            control_snapshot = controller.update(
                measured_pixel_x,
                measured_confidence,
                now,
            )
            if (
                motor is not None
                and motor.healthy
                and not motor_fault_handled
            ):
                motor.request_target(
                    control_snapshot["motor_target_pulses"],
                    now,
                )

            elapsed = now - previous_time
            previous_time = now
            if elapsed > 1e-6:
                instant_fps = 1.0 / elapsed
                fps = (
                    instant_fps
                    if frame_count == 0
                    else 0.90 * fps + 0.10 * instant_fps
                )

            for index, track in enumerate(predicted_tracks):
                _draw_track(frame, track, index)

            frame.draw_rect(
                ROI_X,
                ROI_Y,
                ROI_WIDTH,
                ROI_HEIGHT,
                color=image.COLOR_YELLOW,
                thickness=1,
            )
            for mark_x, mark_color in (
                (int(POSITIVE_50_PX), image.COLOR_RED),
                (int(CENTER_0_PX), image.COLOR_GREEN),
                (int(NEGATIVE_50_PX), image.COLOR_BLUE),
            ):
                frame.draw_line(
                    mark_x,
                    0,
                    mark_x,
                    FRAME_HEIGHT - 1,
                    color=mark_color,
                    thickness=1,
                )

            frame.draw_string(
                6,
                5,
                "{} {:.1f}s {:.1f}fps".format(
                    control_snapshot["state"],
                    control_snapshot["elapsed_s"],
                    fps,
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )
            frame.draw_string(
                6,
                23,
                "x:{:+.1f} v:{:+.0f}".format(
                    control_snapshot["position_mm"],
                    control_snapshot["velocity_mm_s"],
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )
            frame.draw_string(
                6,
                41,
                "ref:{:+.1f} a:{:+.2f} m:{:+d}".format(
                    control_snapshot["target_mm"],
                    control_snapshot["requested_beam_angle_deg"],
                    control_snapshot["motor_target_pulses"],
                ),
                color=image.COLOR_GREEN,
                scale=1.0,
            )
            if motor_error:
                frame.draw_string(
                    6,
                    59,
                    "MOTOR INIT ERROR",
                    color=image.COLOR_RED,
                    scale=1.0,
                )

            if now - last_log_time >= LOG_INTERVAL_SECONDS:
                raw_x = (
                    -1.0
                    if measured_pixel_x is None
                    else measured_pixel_x
                )
                motor_target = (
                    0
                    if motor is None
                    else motor.target_offset_pulses
                )
                ack_count = 0 if motor is None else motor.ack_count
                error_count = 0 if motor is None else motor.error_count
                print(
                    "state={} t={:.2f}s raw_x={:.1f} "
                    "x={:+.1f}mm v={:+.1f}mm/s ref={:+.1f}mm "
                    "rv={:+.1f} ra={:+.0f} "
                    "angle={:+.2f}deg i={:+.2f}deg "
                    "cmd={:+d} lim={} boost={}/{} sent={:+d} "
                    "ack={} err={} rej={} fault={} fps={:.1f}".format(
                        control_snapshot["state"],
                        control_snapshot["elapsed_s"],
                        raw_x,
                        control_snapshot["position_mm"],
                        control_snapshot["velocity_mm_s"],
                        control_snapshot["target_mm"],
                        control_snapshot["reference_velocity_mm_s"],
                        control_snapshot["reference_acceleration_mm_s2"],
                        control_snapshot["requested_beam_angle_deg"],
                        control_snapshot["integral_angle_deg"],
                        control_snapshot["motor_target_pulses"],
                        control_snapshot["drive_limit_pulses"],
                        1 if control_snapshot["stiction_active"] else 0,
                        control_snapshot["stiction_event_count"],
                        motor_target,
                        ack_count,
                        error_count,
                        control_snapshot["vision_reject_count"],
                        (
                            "-"
                            if not control_snapshot["fault_reason"]
                            else control_snapshot["fault_reason"].replace(
                                " ",
                                "_",
                            )
                        ),
                        fps,
                    )
                )
                last_log_time = now

            disp.show(frame)
            frame_count += 1
    finally:
        if motor is not None:
            motor.level_and_stop()
            motor.close()
        cam.close()


if __name__ == "__main__":
    main()
