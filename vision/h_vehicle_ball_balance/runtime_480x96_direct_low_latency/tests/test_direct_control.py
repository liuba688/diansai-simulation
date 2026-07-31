import pathlib
import sys
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from ball_control import BallTaskController, pixel_to_mm
from x42s_uart import (
    X42SMotor,
    build_enable_frame,
    build_position_frame,
    build_stop_frame,
)


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.rx = bytearray()
        self.closed = False

    def write(self, frame):
        frame = bytes(frame)
        self.writes.append(frame)
        self.rx.extend([0x01, frame[1], 0x02, 0x6B])
        return len(frame)

    def read(self):
        data = bytes(self.rx)
        self.rx.clear()
        return data

    def close(self):
        self.closed = True


class CalibrationTests(unittest.TestCase):
    def test_three_calibration_points(self):
        self.assertAlmostEqual(pixel_to_mm(212), 50.0)
        self.assertAlmostEqual(pixel_to_mm(324), 0.0)
        self.assertAlmostEqual(pixel_to_mm(435), -50.0)

    def test_piecewise_direction(self):
        self.assertGreater(pixel_to_mm(250), 0.0)
        self.assertLess(pixel_to_mm(380), 0.0)


class ProtocolTests(unittest.TestCase):
    def test_enable_frame(self):
        self.assertEqual(
            build_enable_frame(True),
            bytes([0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B]),
        )

    def test_relative_position_frame(self):
        self.assertEqual(
            build_position_frame(-100, movement_mode=0),
            bytes(
                [
                    0x01,
                    0xFD,
                    0x00,
                    0x00,
                    0x1E,
                    0x64,
                    0x00,
                    0x00,
                    0x00,
                    0x64,
                    0x00,
                    0x00,
                    0x6B,
                ]
            ),
        )
        self.assertEqual(build_position_frame(1)[2], 0x01)

    def test_stop_frame(self):
        self.assertEqual(
            build_stop_frame(),
            bytes([0x01, 0xFE, 0x98, 0x00, 0x6B]),
        )

    def test_motor_startup_and_slew_limited_target(self):
        serial = FakeSerial()
        motor = X42SMotor(serial_port=serial)
        self.assertTrue(motor.initialize())
        self.assertEqual(serial.writes[0][1], 0xF3)
        self.assertEqual(serial.writes[1][1], 0xFD)
        self.assertEqual(serial.writes[1][10], 0x02)

        motor._last_send_time = 0.0
        self.assertTrue(motor.request_target(-100, now=1.0))
        self.assertEqual(serial.writes[-1][2], 0x00)
        self.assertEqual(motor.target_offset_pulses, -48)
        self.assertTrue(motor.poll(now=1.01))


class StateMachineTests(unittest.TestCase):
    def test_wait_center_then_positive_and_negative(self):
        controller = BallTaskController()
        now = 0.0
        for _ in range(8):
            controller.update(324.0, 0.9, now)
            now += 0.1
        self.assertEqual(controller.state, "TO_POSITIVE")

        controller.target_mm = 50.0
        for _ in range(10):
            controller.update(212.0, 0.9, now)
            now += 0.1
        self.assertEqual(controller.state, "TO_NEGATIVE")

        controller.target_mm = -50.0
        for _ in range(8):
            controller.estimator.velocity_mm_s = 0.0
            controller.update(435.0, 0.9, now)
            now += 0.1
        self.assertEqual(controller.state, "HOLD_NEGATIVE")

    def test_motor_sign_matches_mechanism(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.target_mm = 50.0
        snapshot = controller.update(324.0, 0.9, 0.1)
        self.assertGreater(snapshot["motor_target_pulses"], 0)

    def test_positive_endpoint_waits_for_low_speed_before_reversal(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.target_mm = 50.0
        controller.target_goal_mm = 50.0

        controller._update_state(48.0, 90.0, True, 0.5)
        self.assertEqual(controller.state, "TO_POSITIVE")
        self.assertIsNone(controller.positive_stable_since)

        controller._update_state(48.0, 0.0, True, 0.6)
        controller._update_state(48.0, 0.0, True, 0.85)
        self.assertEqual(controller.state, "TO_NEGATIVE")

    def test_positive_phase_is_not_forced_to_timeout(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.target_goal_mm = 50.0
        controller.target_mm = 50.0
        controller._update_state(40.0, 0.0, True, 8.0)
        self.assertEqual(controller.state, "TO_POSITIVE")

    def test_linkage_positive_drive_has_separate_limit(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.target_goal_mm = 50.0
        controller.target_mm = 50.0
        output = controller._calculate_motor_target(
            position_mm=0.0,
            velocity_mm_s=0.0,
            valid=True,
            dt=0.02,
        )
        self.assertEqual(output, 75)
        self.assertEqual(controller.active_drive_limit_pulses, 75)

    def test_wrong_direction_enters_fault(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.update(324.0, 0.9, 0.0)
        snapshot = controller.update(360.0, 0.9, 0.5)
        self.assertEqual(snapshot["state"], "FAULT")
        self.assertEqual(snapshot["fault_reason"], "direction mismatch")
        self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_ball_edge_enters_fault(self):
        controller = BallTaskController()
        controller.state = "TO_NEGATIVE"
        controller.start_time = 0.0
        snapshot = controller.update(580.0, 0.9, 0.5)
        self.assertEqual(snapshot["state"], "FAULT")
        self.assertEqual(snapshot["fault_reason"], "ball edge")

    def test_negative_reference_uses_slower_slew(self):
        controller = BallTaskController()
        controller.state = "TO_NEGATIVE"
        controller.start_time = 0.0
        controller.target_mm = 50.0
        controller.last_update_time = 0.0
        snapshot = controller.update(212.0, 0.9, 0.1)
        self.assertAlmostEqual(snapshot["target_mm"], 43.0)

    def test_negative_drive_has_asymmetric_limit(self):
        controller = BallTaskController()
        controller.state = "TO_NEGATIVE"
        controller.start_time = 0.0
        controller.target_mm = -50.0
        snapshot = controller.update(244.0, 0.9, 0.1)
        self.assertEqual(snapshot["motor_target_pulses"], -100)

    def test_linkage_near_target_tapers_drive_but_keeps_braking(self):
        controller = BallTaskController()
        controller.state = "TO_NEGATIVE"
        controller.start_time = 0.0
        controller.target_goal_mm = -50.0
        controller.target_mm = -50.0

        drive = controller._calculate_motor_target(
            position_mm=-44.0,
            velocity_mm_s=30.0,
            valid=True,
            dt=0.02,
        )
        self.assertEqual(drive, -40)
        self.assertEqual(controller.active_drive_limit_pulses, 40)

        brake = controller._calculate_motor_target(
            position_mm=-44.0,
            velocity_mm_s=-100.0,
            valid=True,
            dt=0.02,
        )
        self.assertEqual(brake, 120)

    def test_linkage_hold_output_is_limited(self):
        controller = BallTaskController()
        controller.state = "HOLD_NEGATIVE"
        controller.start_time = 0.0
        controller.target_goal_mm = -50.0
        controller.target_mm = -50.0
        output = controller._calculate_motor_target(
            position_mm=-20.0,
            velocity_mm_s=0.0,
            valid=True,
            dt=0.02,
        )
        self.assertEqual(output, -45)
        self.assertEqual(controller.active_drive_limit_pulses, 45)

    def test_entering_hold_clears_integral(self):
        controller = BallTaskController()
        controller.state = "TO_NEGATIVE"
        controller.start_time = 0.0
        controller.target_mm = -50.0
        controller.target_goal_mm = -50.0
        controller.integral_mm_s = -100.0
        controller.negative_stable_since = 0.0
        controller._update_state(-50.0, 0.0, True, 0.5)
        self.assertEqual(controller.state, "HOLD_NEGATIVE")
        self.assertEqual(controller.integral_mm_s, 0.0)

if __name__ == "__main__":
    unittest.main()
