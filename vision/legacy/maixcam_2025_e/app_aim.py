"""2025 E 视觉应用顶层。"""

import time

import cv2
import numpy as np
from maix import app, camera, display, image

import config
from target_detector import TargetDetector
from tracking_filter import STATUS_HOLD, STATUS_SEARCH, STATUS_TRACKING, TargetTracker, TrackState
from vision_protocol import VisionUartLink


_clock = getattr(time, "monotonic", time.time)


def _draw_cross(image_bgr, x, y, size, color, thickness=1):
    cv2.line(image_bgr, (x - size, y), (x + size, y), color, thickness)
    cv2.line(image_bgr, (x, y - size), (x, y + size), color, thickness)


class AimVisionApp:
    def __init__(self):
        self._camera = camera.Camera(config.FRAME_WIDTH, config.FRAME_HEIGHT)
        self._display = display.Display()
        self._detector = TargetDetector()
        self._tracker = TargetTracker()
        self._uart = VisionUartLink()
        self._last_track = TrackState()
        self._frame_count = 0
        self._last_log_time = 0.0
        self._last_frame_time = _clock()
        self._fps = 0.0

    def _draw_overlay(self, image_bgr, track):
        _draw_cross(
            image_bgr,
            config.AIM_REFERENCE_X,
            config.AIM_REFERENCE_Y,
            8,
            (0, 255, 0),
            2,
        )
        if track.valid and track.corners is not None:
            color = (0, 0, 255)
            if track.status == STATUS_HOLD:
                color = (0, 255, 255)
            elif track.status == STATUS_SEARCH:
                color = (255, 128, 0)
            corners = np.asarray(track.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image_bgr, [corners], True, color, 2)
            cv2.circle(image_bgr, (track.center_x, track.center_y), 3, (255, 0, 0), -1)

        names = {STATUS_TRACKING: "TARGET", STATUS_HOLD: "HOLD", STATUS_SEARCH: "SEARCH"}
        name = names.get(track.status, "LOST")
        cv2.putText(
            image_bgr,
            "{} dx:{:+d} dy:{:+d} conf:{}".format(name, track.dx, track.dy, track.confidence),
            (5, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 0) if track.status == STATUS_TRACKING else (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image_bgr,
            "fps:{:.1f}".format(self._fps),
            (5, config.FRAME_HEIGHT - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _log(self, now, track):
        if now - self._last_log_time < config.LOG_INTERVAL_S:
            return
        self._last_log_time = now
        print(
            "status={} x={} y={} dx={} dy={} conf={} fps={:.1f}".format(
                track.status,
                track.center_x,
                track.center_y,
                track.dx,
                track.dy,
                track.confidence,
                self._fps,
            )
        )

    def run(self):
        print("2025 E phase 1: A4 black-frame detection")
        print("Laser and motor output are disabled")
        try:
            while not app.need_exit():
                frame = self._camera.read()
                image_bgr = image.image2cv(frame, ensure_bgr=True, copy=False)
                if self._frame_count % config.PROCESS_EVERY_N_FRAMES == 0:
                    detection = self._detector.detect(image_bgr)
                    self._last_track = self._tracker.update(detection)
                    self._uart.send_track(self._last_track)

                now = _clock()
                elapsed = now - self._last_frame_time
                self._last_frame_time = now
                if elapsed > 1e-6:
                    instant_fps = 1.0 / elapsed
                    self._fps = instant_fps if self._fps <= 0.0 else 0.9 * self._fps + 0.1 * instant_fps
                self._draw_overlay(image_bgr, self._last_track)
                self._log(now, self._last_track)
                self._display.show(image.cv2image(image_bgr, copy=False))
                self._frame_count += 1
        finally:
            self._uart.close()
            self._camera.close()
