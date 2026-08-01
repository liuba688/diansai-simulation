"""2026 H resident MaixCAM vision process.

The camera and model are initialized once. MSPM0 MODE_SELECT/START/STOP
messages switch tasks without restarting this application. The MSPM0 remains
the only start-time owner and directly controls the X42S.
"""

SOURCE_VERSION = "1.0.1-heartbeat-fix"

import os
import time
from maix import app, camera, display, err, image, nn, pinmap, uart

from ball_estimator import (BallEstimator, CENTER_0_PX, NEGATIVE_50_PX,
                            POSITIVE_50_PX)
from mission_protocol import (BALL_CONFIRMED, BALL_VALID, CAM_HEARTBEAT,
                              FAULT, MCU_HEARTBEAT, MODE_READY, MODE_SELECT,
                              START, STARTED, STOP, Parser, build_ball,
                              build_event, decode_command)

MODEL_NAME = "pipe_ball_480x96_v1.mud"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), MODEL_NAME)
FRAME_WIDTH = 640
FRAME_HEIGHT = 128
CAMERA_FPS = 60
DETECT_CONFIDENCE = 0.45
DISPLAY_CONFIDENCE = 0.50
IOU_THRESHOLD = 0.35
MIN_BOX_SIDE = 8
MIN_ASPECT = 0.70
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
BALL_SEND_PERIOD_S = 1.0 / 30.0
HEARTBEAT_PERIOD_S = 0.20
# The MSPM0 transmits a heartbeat every 200 ms.  Keep enough margin for
# inference/display jitter while still stopping quickly after a real link loss.
MCU_TIMEOUT_S = 1.20
READY_VALID_FRAMES = 3

STATE_IDLE = 0
STATE_CONFIGURING = 1
STATE_READY = 2
STATE_RUNNING = 3
STATE_LINK_FAULT = 4

clock = getattr(time, "monotonic", time.time)


class MissionLink:
    def __init__(self, serial_port):
        self.serial = serial_port
        self.parser = Parser()
        self.sequence = 0
        self.state = STATE_IDLE
        self.task_id = 0
        self.run_id = 0
        self.target_x10_mm = 0
        self.speed_tier = 0
        self.valid_frames = 0
        self.last_mcu_time = clock()
        self.last_ball_send = 0.0
        self.last_heartbeat = 0.0
        self.fault_sent = False

    def send(self, frame):
        written = self.serial.write(frame)
        if written is not None and int(written) < len(frame):
            raise RuntimeError("UART short write")

    def send_event(self, message_type, result=0):
        self.send(build_event(message_type, self.sequence, self.task_id,
                              self.run_id, result, self.state))
        self.sequence = (self.sequence + 1) & 0xFF
        if message_type != CAM_HEARTBEAT:
            self.last_heartbeat = clock()

    def handle_command(self, command, estimator):
        self.last_mcu_time = clock()
        kind = command["type"]
        if kind == MODE_SELECT:
            if (command["task_id"] == self.task_id
                    and command["run_id"] == self.run_id
                    and self.state in (STATE_READY, STATE_RUNNING)):
                self.send_event(MODE_READY, 0)
                return
            self.task_id = command["task_id"]
            self.run_id = command["run_id"]
            self.target_x10_mm = command["target_x10_mm"]
            self.speed_tier = command["speed_tier"]
            self.valid_frames = 0
            self.state = STATE_CONFIGURING
            self.fault_sent = False
            estimator.reset()
            if self.task_id in (1, 2):
                self.state = STATE_READY
                self.send_event(MODE_READY, 0)
        elif kind == START:
            if (command["task_id"] == self.task_id
                    and command["run_id"] == self.run_id
                    and self.state in (STATE_READY, STATE_RUNNING)):
                self.state = STATE_RUNNING
                self.send_event(STARTED, 0)
        elif kind == STOP:
            if command["run_id"] == self.run_id:
                self.state = STATE_IDLE
                self.valid_frames = 0
        elif kind == MCU_HEARTBEAT:
            pass

    def poll(self, estimator):
        data = self.serial.read()
        for packet in self.parser.feed(data):
            command = decode_command(packet)
            if command is not None:
                self.handle_command(command, estimator)

    def observe_valid_ball(self):
        if self.state != STATE_CONFIGURING or self.task_id not in (3, 4, 5, 6):
            return
        self.valid_frames += 1
        if self.valid_frames >= READY_VALID_FRAMES:
            self.state = STATE_READY
            self.send_event(MODE_READY, 0)

    def update_tx(self, estimate, now):
        position, velocity, age, valid, confidence = estimate
        if now - self.last_mcu_time > MCU_TIMEOUT_S and self.state == STATE_RUNNING:
            self.state = STATE_LINK_FAULT
            if not self.fault_sent:
                self.send_event(FAULT, 1)
                self.fault_sent = True
        if now - self.last_heartbeat >= HEARTBEAT_PERIOD_S:
            self.send_event(CAM_HEARTBEAT, 0)
            self.last_heartbeat = now
        if self.task_id in (3, 4, 5, 6) and now - self.last_ball_send >= BALL_SEND_PERIOD_S:
            flags = (BALL_VALID | BALL_CONFIRMED) if valid else 0
            frame = build_ball(
                self.sequence, self.task_id, self.run_id, flags,
                int(round(position * 10.0)), int(round(velocity)),
                int(round(confidence * 255.0)), int(round(age * 1000.0)),
                int(round(now * 1000.0)))
            self.send(frame)
            self.sequence = (self.sequence + 1) & 0xFF
            self.last_ball_send = now


def select_ball(detections):
    candidates = []
    for detection in detections:
        score = float(detection.score)
        width = int(detection.w)
        height = int(detection.h)
        if score < DETECT_CONFIDENCE or width < MIN_BOX_SIDE or height < MIN_BOX_SIDE:
            continue
        aspect = width / float(max(1, height))
        if not MIN_ASPECT <= aspect <= 1.0 / MIN_ASPECT:
            continue
        candidates.append({
            "x": int(detection.x), "y": int(detection.y),
            "w": width, "h": height, "score": score,
            "cx": float(detection.x) + width * 0.5,
        })
    return max(candidates, key=lambda item: item["score"]) if candidates else None


def main():
    if not os.path.exists(MODEL_PATH):
        raise RuntimeError("model missing: {}".format(MODEL_PATH))
    detector = nn.YOLOv8(model=MODEL_PATH, dual_buff=False)
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, detector.input_format(),
                        fps=CAMERA_FPS, buff_num=1)
    disp = display.Display()
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "map A19")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "map A18")
    serial_port = uart.UART(UART_DEVICE, UART_BAUDRATE)
    link = MissionLink(serial_port)
    estimator = BallEstimator()
    previous = clock()
    fps = 0.0

    print("H integrated vision {} UART1 115200".format(SOURCE_VERSION))
    print("calibration +50={} O={} -50={}".format(
        POSITIVE_50_PX, CENTER_0_PX, NEGATIVE_50_PX))
    try:
        while not app.need_exit():
            link.poll(estimator)
            frame = cam.read()
            detections = detector.detect(frame, conf_th=DETECT_CONFIDENCE,
                                         iou_th=IOU_THRESHOLD,
                                         fit=image.Fit.FIT_CONTAIN)
            ball = select_ball(detections)
            now = clock()
            if ball is not None:
                accepted = estimator.update(ball["cx"], ball["score"], now)
                if accepted:
                    link.observe_valid_ball()
                color = image.COLOR_GREEN if ball["score"] >= DISPLAY_CONFIDENCE else image.COLOR_RED
                frame.draw_rect(ball["x"], ball["y"], ball["w"], ball["h"],
                                color=color, thickness=2)
            position, velocity, age, valid = estimator.estimate(now)
            link.update_tx((position, velocity, age, valid, estimator.confidence), now)

            dt = max(0.0001, now - previous)
            previous = now
            fps = 1.0 / dt if fps == 0.0 else 0.9 * fps + 0.1 / dt
            for mark, color in ((POSITIVE_50_PX, image.COLOR_RED),
                                (CENTER_0_PX, image.COLOR_GREEN),
                                (NEGATIVE_50_PX, image.COLOR_BLUE)):
                frame.draw_line(int(mark), 0, int(mark), FRAME_HEIGHT - 1,
                                color=color, thickness=1)
            frame.draw_string(5, 4, "H v{} T{} R{} S{}".format(
                SOURCE_VERSION, link.task_id, link.run_id, link.state),
                color=image.COLOR_GREEN, scale=1.0)
            frame.draw_string(5, 23, "x:{:+.1f} v:{:+.0f} age:{:.0f}".format(
                position, velocity, age * 1000.0),
                color=image.COLOR_GREEN if valid else image.COLOR_RED, scale=1.0)
            frame.draw_string(5, 42, "fps:{:.1f} uart crc:{} fmt:{}".format(
                fps, link.parser.crc_errors, link.parser.format_errors),
                color=image.COLOR_GREEN, scale=1.0)
            disp.show(frame)
    finally:
        try:
            serial_port.close()
        finally:
            cam.close()


if __name__ == "__main__":
    main()
