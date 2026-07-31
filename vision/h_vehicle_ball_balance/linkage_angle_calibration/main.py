"""Touch-operated X42S linkage angle calibration.

Remove the steel ball before running this app.
"""

SOURCE_VERSION = "0.2.0"

import time

from maix import app, display, image, touchscreen

from linkage_geometry import (
    geometry_summary,
    motor_deg_from_pulses,
    theoretical_beam_deg_from_pulses,
)
from x42s_uart import X42SMotor


BACKGROUND = image.Color.from_rgb(12, 18, 28)
WHITE = image.Color.from_rgb(235, 240, 248)
CYAN = image.Color.from_rgb(60, 210, 235)
GREEN = image.Color.from_rgb(70, 210, 120)
YELLOW = image.Color.from_rgb(245, 195, 60)
RED = image.Color.from_rgb(245, 75, 75)
GRAY = image.Color.from_rgb(80, 92, 108)

TARGETS = (-40, -20, 0, 20, 40)
EXTENDED_TARGETS = (-160, -140, -120, -100, 100, 120, 140, 160)
UNLOCK_HOLD_S = 1.5
QUERY_PERIOD_S = 0.35
SETTLE_QUERY_DELAY_S = 0.20
_clock = getattr(time, "monotonic", time.time)


def _inside(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def _draw_button(canvas, rect, label, color, active=False):
    x, y, w, h = rect
    canvas.draw_rect(x, y, w, h, color=color, thickness=-1)
    if active:
        canvas.draw_rect(x, y, w, h, color=WHITE, thickness=3)
    canvas.draw_string(
        x + 8,
        y + (h - 18) // 2,
        label,
        color=WHITE,
        scale=1.2,
    )


def main():
    disp = display.Display()
    touch = touchscreen.TouchScreen()
    motor = X42SMotor()

    desired = 0
    stopped = False
    unlocked = False
    unlock_started = None
    last_pressed = False
    last_query = 0.0
    target_reached_at = None
    boot_motor_deg = None
    actual_relative_deg = None
    status = "REMOVE BALL - initializing"

    width = disp.width()
    height = disp.height()
    button_y = max(118, height - 155)
    gap = 5
    button_w = max(55, (width - gap * 6) // 5)
    target_rects = []
    for index, target in enumerate(TARGETS):
        target_rects.append(
            (
                gap + index * (button_w + gap),
                button_y,
                button_w,
                42,
                target,
            )
        )
    extended_button_w = max(48, (width - gap * 9) // 8)
    extended_rects = []
    for index, target in enumerate(EXTENDED_TARGETS):
        extended_rects.append(
            (
                gap + index * (extended_button_w + gap),
                button_y + 50,
                extended_button_w,
                42,
                target,
            )
        )
    unlock_rect = (gap, button_y + 100, 235, 42)
    stop_rect = (width - 105, button_y + 100, 100, 42)

    try:
        motor.initialize()
        boot_motor_deg = motor.read_current_position_deg()
        status = "READY - tap small target first"
        print(
            "linkage calibration v{} ready; {}; boot_deg={}".format(
                SOURCE_VERSION,
                geometry_summary(),
                boot_motor_deg,
            )
        )

        while not app.need_exit():
            now = _clock()
            x, y, pressed = touch.read()
            just_pressed = pressed and not last_pressed

            if stopped:
                status = "STOPPED - power cycle to resume"
            elif just_pressed:
                for rx, ry, rw, rh, target in target_rects:
                    if _inside(x, y, (rx, ry, rw, rh)):
                        desired = target
                        target_reached_at = None
                        status = "target {:+d} pulses".format(desired)
                        break
                if _inside(x, y, stop_rect):
                    motor.emergency_stop()
                    stopped = True
                elif unlocked:
                    for rx, ry, rw, rh, target in extended_rects:
                        if _inside(x, y, (rx, ry, rw, rh)):
                            desired = target
                            target_reached_at = None
                            status = "EXT target {:+d} pulses".format(desired)
                            break

            if not stopped and pressed and _inside(x, y, unlock_rect):
                if unlock_started is None:
                    unlock_started = now
                elif now - unlock_started >= UNLOCK_HOLD_S:
                    unlocked = True
                    status = "extended range unlocked"
            else:
                unlock_started = None

            if not stopped:
                motor.poll(now)
                motor.request_target(desired, now)
                if motor.target_offset_pulses == desired:
                    if target_reached_at is None:
                        target_reached_at = now
                    if (
                        boot_motor_deg is not None
                        and now - target_reached_at >= SETTLE_QUERY_DELAY_S
                        and now - last_query >= QUERY_PERIOD_S
                    ):
                        current_deg = motor.read_current_position_deg()
                        last_query = now
                        if current_deg is not None:
                            actual_relative_deg = current_deg - boot_motor_deg
                else:
                    target_reached_at = None

            canvas = image.Image(width, height, image.Format.FMT_RGB888)
            canvas.draw_rect(0, 0, width, height, color=BACKGROUND, thickness=-1)
            canvas.draw_string(
                8,
                6,
                "LINKAGE CAL v{}   BALL REMOVED".format(SOURCE_VERSION),
                color=YELLOW,
                scale=1.2,
            )
            canvas.draw_string(8, 32, geometry_summary(), color=CYAN, scale=1.0)
            canvas.draw_string(
                8,
                54,
                "target={:+d}  sent={:+d} pulse  theory={:+.3f} deg".format(
                    desired,
                    motor.target_offset_pulses,
                    theoretical_beam_deg_from_pulses(
                        motor.target_offset_pulses
                    ),
                ),
                color=WHITE,
                scale=1.0,
            )
            actual_text = (
                "actual motor delta: unavailable"
                if actual_relative_deg is None
                else "actual motor delta={:+.3f} deg".format(
                    actual_relative_deg
                )
            )
            canvas.draw_string(8, 76, actual_text, color=GREEN, scale=1.0)
            canvas.draw_string(
                8,
                98,
                "{}  ack={} err={}".format(
                    status,
                    motor.ack_count,
                    motor.error_count,
                ),
                color=RED if stopped or not motor.healthy else WHITE,
                scale=0.9,
            )

            for rx, ry, rw, rh, target in target_rects:
                _draw_button(
                    canvas,
                    (rx, ry, rw, rh),
                    "{:+d}".format(target),
                    GREEN if target == 0 else GRAY,
                    desired == target,
                )
            for rx, ry, rw, rh, target in extended_rects:
                _draw_button(
                    canvas,
                    (rx, ry, rw, rh),
                    "{:+d}".format(target),
                    CYAN if unlocked else GRAY,
                    desired == target,
                )
            hold_text = (
                "EXTENDED +/-160 UNLOCKED"
                if unlocked
                else "HOLD 1.5s: UNLOCK +/-100..160"
            )
            _draw_button(
                canvas,
                unlock_rect,
                hold_text,
                GREEN if unlocked else YELLOW,
                pressed and _inside(x, y, unlock_rect),
            )
            _draw_button(canvas, stop_rect, "STOP", RED, stopped)
            disp.show(canvas)
            last_pressed = pressed
            time.sleep(0.010)
    finally:
        motor.emergency_stop()
        motor.close()


if __name__ == "__main__":
    main()
