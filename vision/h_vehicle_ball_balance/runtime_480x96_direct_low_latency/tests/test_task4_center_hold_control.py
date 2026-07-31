import math
import pathlib
import sys
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from task4_center_hold_control import (  # noqa: E402
    BallEstimator,
    BallTaskController,
    HOLD_MAX_BEAM_ANGLE_DEG,
    NEGATIVE_STICTION_MIN_ANGLE_DEG,
    NEGATIVE_STICTION_MAX_ANGLE_DEG,
    POSITIVE_STICTION_MIN_ANGLE_DEG,
    POSITIVE_STICTION_MAX_ANGLE_DEG,
    RECOVERY_MAX_BEAM_ANGLE_DEG,
    beam_angle_to_motor_pulses,
    pixel_to_mm,
)


class CalibrationTests(unittest.TestCase):
    def test_current_three_point_calibration(self):
        self.assertAlmostEqual(pixel_to_mm(212.0), 50.0)
        self.assertAlmostEqual(pixel_to_mm(324.0), 0.0)
        self.assertAlmostEqual(pixel_to_mm(435.0), -50.0)

    def test_measured_linkage_map(self):
        self.assertEqual(beam_angle_to_motor_pulses(0.0), 0)
        self.assertEqual(beam_angle_to_motor_pulses(1.0), 80)
        self.assertEqual(beam_angle_to_motor_pulses(-1.0), -80)
        self.assertEqual(beam_angle_to_motor_pulses(2.2), 128)
        self.assertEqual(beam_angle_to_motor_pulses(4.0), 200)

    def test_relaxed_control_limits_stay_inside_mechanical_limit(self):
        self.assertEqual(
            beam_angle_to_motor_pulses(HOLD_MAX_BEAM_ANGLE_DEG),
            168,
        )
        self.assertEqual(
            beam_angle_to_motor_pulses(RECOVERY_MAX_BEAM_ANGLE_DEG),
            196,
        )
        self.assertLessEqual(
            beam_angle_to_motor_pulses(RECOVERY_MAX_BEAM_ANGLE_DEG),
            200,
        )


class CenterHoldTests(unittest.TestCase):
    @staticmethod
    def arm(controller):
        now = 0.0
        for _ in range(8):
            controller.update(324.0, 0.9, now)
            now += 0.1
        return now

    def test_arms_and_holds_zero_without_starting_endpoint_task(self):
        controller = BallTaskController()
        now = self.arm(controller)
        self.assertEqual(controller.state, "HOLD_CENTER")
        for _ in range(30):
            snapshot = controller.update(324.0, 0.9, now)
            now += 0.1
        self.assertEqual(snapshot["state"], "HOLD_CENTER")
        self.assertEqual(snapshot["target_mm"], 0.0)
        self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_arms_from_far_positive_position_and_recovers(self):
        controller = BallTaskController()
        now = 0.0
        snapshot = None
        for _ in range(6):
            snapshot = controller.update(145.0, 0.9, now)
            now += 0.05
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertGreater(snapshot["position_mm"], 70.0)
        self.assertLess(snapshot["motor_target_pulses"], 0)

    def test_arms_from_far_negative_position_and_recovers(self):
        controller = BallTaskController()
        now = 0.0
        snapshot = None
        for _ in range(6):
            snapshot = controller.update(500.0, 0.9, now)
            now += 0.05
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertLess(snapshot["position_mm"], -70.0)
        self.assertGreater(snapshot["motor_target_pulses"], 0)

    def test_near_end_position_still_recovers_instead_of_faulting(self):
        controller = BallTaskController()
        now = 0.0
        snapshot = None
        for _ in range(10):
            snapshot = controller.update(100.0, 0.9, now)
            now += 0.05
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertGreater(snapshot["position_mm"], 95.0)
        self.assertLess(snapshot["motor_target_pulses"], 0)

    def test_positive_disturbance_commands_negative_beam(self):
        controller = BallTaskController()
        now = self.arm(controller)
        controller.update(280.0, 0.9, now)
        snapshot = controller.update(280.0, 0.9, now + 0.05)
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertLess(snapshot["requested_beam_angle_deg"], 0.0)
        self.assertLess(snapshot["motor_target_pulses"], 0)
        self.assertTrue(snapshot["out_of_spec"])

    def test_negative_disturbance_commands_positive_beam(self):
        controller = BallTaskController()
        now = self.arm(controller)
        controller.update(370.0, 0.9, now)
        snapshot = controller.update(370.0, 0.9, now + 0.05)
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertGreater(snapshot["requested_beam_angle_deg"], 0.0)
        self.assertGreater(snapshot["motor_target_pulses"], 0)

    def test_negative_10_mm_stall_gets_positive_breakaway_angle(self):
        controller = BallTaskController()
        controller.state = "RECOVER_CENTER"
        controller.start_time = 0.0
        controller.recovery_since = 0.0
        pixel_x = 324.0 + 10.0 * (435.0 - 324.0) / 50.0
        snapshot = None
        for index in range(5):
            snapshot = controller.update(pixel_x, 0.9, index * 0.05)
        self.assertTrue(snapshot["stiction_active"])
        self.assertGreaterEqual(
            snapshot["requested_beam_angle_deg"],
            POSITIVE_STICTION_MIN_ANGLE_DEG,
        )

    def test_positive_10_mm_stall_gets_negative_breakaway_angle(self):
        controller = BallTaskController()
        controller.state = "RECOVER_CENTER"
        controller.start_time = 0.0
        controller.recovery_since = 0.0
        pixel_x = 324.0 - 10.0 * (324.0 - 212.0) / 50.0
        snapshot = None
        for index in range(5):
            snapshot = controller.update(pixel_x, 0.9, index * 0.05)
        self.assertTrue(snapshot["stiction_active"])
        self.assertLessEqual(
            snapshot["requested_beam_angle_deg"],
            -NEGATIVE_STICTION_MIN_ANGLE_DEG,
        )

    def test_stalled_breakaway_ramps_to_directional_caps(self):
        for pixel_x, expected_sign, expected_cap in (
            (
                324.0 + 10.0 * (435.0 - 324.0) / 50.0,
                1.0,
                POSITIVE_STICTION_MAX_ANGLE_DEG,
            ),
            (
                324.0 - 10.0 * (324.0 - 212.0) / 50.0,
                -1.0,
                NEGATIVE_STICTION_MAX_ANGLE_DEG,
            ),
        ):
            controller = BallTaskController()
            controller.state = "RECOVER_CENTER"
            controller.start_time = 0.0
            controller.recovery_since = 0.0
            snapshot = None
            for index in range(30):
                snapshot = controller.update(pixel_x, 0.9, index * 0.05)
            self.assertTrue(snapshot["stiction_active"])
            self.assertAlmostEqual(
                snapshot["requested_beam_angle_deg"],
                expected_sign * expected_cap,
            )

    def test_recovery_requires_centered_low_speed_settle(self):
        controller = BallTaskController()
        now = self.arm(controller)
        controller.update(280.0, 0.9, now)
        controller.update(280.0, 0.9, now + 0.05)
        self.assertEqual(controller.state, "RECOVER_CENTER")

        now += 0.10
        for _ in range(12):
            controller.estimator.velocity_mm_s = 0.0
            snapshot = controller.update(324.0, 0.9, now)
            now += 0.05
        self.assertEqual(snapshot["state"], "HOLD_CENTER")
        self.assertEqual(snapshot["recovery_count"], 1)
        self.assertGreater(snapshot["last_recovery_time_s"], 0.0)

    def test_vision_timeout_searches_and_levels(self):
        controller = BallTaskController()
        now = self.arm(controller)
        snapshot = controller.update(None, 0.0, now + 0.2)
        self.assertEqual(snapshot["state"], "SEARCH_BALL")
        self.assertEqual(snapshot["fault_reason"], "")
        self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_lost_ball_is_reacquired_and_returned_to_center(self):
        controller = BallTaskController()
        now = self.arm(controller)
        snapshot = controller.update(None, 0.0, now + 0.2)
        self.assertEqual(snapshot["state"], "SEARCH_BALL")

        now += 0.25
        for _ in range(6):
            snapshot = controller.update(145.0, 0.9, now)
            now += 0.05
        self.assertEqual(snapshot["state"], "RECOVER_CENTER")
        self.assertLess(snapshot["motor_target_pulses"], 0)

    def test_hardware_failure_uses_safe_stop_not_fault(self):
        controller = BallTaskController()
        controller.force_fault("motor response")
        snapshot = controller.update(324.0, 0.9, 0.0)
        self.assertEqual(snapshot["state"], "SAFE_STOP")
        self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_pixel_calibration_never_drives_motor(self):
        controller = BallTaskController(pixel_calibration_only=True)
        for index, pixel_x in enumerate((212.0, 324.0, 435.0)):
            snapshot = controller.update(pixel_x, 0.9, index * 0.1)
            self.assertEqual(snapshot["state"], "PIXEL_CALIBRATION")
            self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_nominal_delayed_friction_plant_recovers_from_20_mm(self):
        controller = BallTaskController()
        now = self.arm(controller)
        dt = 0.01
        position_mm = 20.0
        velocity_mm_s = 0.0
        beam_angle_deg = 0.0
        next_vision_time = now
        snapshot = None

        while now <= 5.0:
            measured_pixel = None
            if now + 1e-9 >= next_vision_time:
                if position_mm >= 0.0:
                    measured_pixel = (
                        324.0 - position_mm * (324.0 - 212.0) / 50.0
                    )
                else:
                    measured_pixel = (
                        324.0 + (-position_mm) * (435.0 - 324.0) / 50.0
                    )
                next_vision_time += 0.05

            snapshot = controller.update(measured_pixel, 0.95, now)
            requested_angle_deg = snapshot["requested_beam_angle_deg"]

            requested_rate_deg_s = (
                requested_angle_deg - beam_angle_deg
            ) / 0.07
            requested_rate_deg_s = max(
                -15.0,
                min(15.0, requested_rate_deg_s),
            )
            beam_angle_deg += requested_rate_deg_s * dt

            gravity_acceleration_mm_s2 = (
                (5.0 / 7.0)
                * 9.81
                * 1000.0
                * math.sin(math.radians(beam_angle_deg))
            )
            if (
                abs(velocity_mm_s) < 1.0
                and abs(gravity_acceleration_mm_s2) < 110.0
            ):
                acceleration_mm_s2 = 0.0
                velocity_mm_s = 0.0
            else:
                friction_direction = (
                    velocity_mm_s
                    if abs(velocity_mm_s) >= 1.0
                    else gravity_acceleration_mm_s2
                )
                acceleration_mm_s2 = gravity_acceleration_mm_s2 - (
                    math.copysign(35.0, friction_direction)
                )

            position_mm += (
                velocity_mm_s * dt
                + 0.5 * acceleration_mm_s2 * dt * dt
            )
            velocity_mm_s += acceleration_mm_s2 * dt
            now += dt

            if (
                snapshot["state"] == "HOLD_CENTER"
                and snapshot["recovery_count"] == 1
                and snapshot["last_recovery_time_s"] > 0.0
            ):
                break

        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot["state"], "HOLD_CENTER")
        self.assertEqual(snapshot["recovery_count"], 1)
        self.assertLessEqual(abs(position_mm), 6.0)
        self.assertLessEqual(abs(velocity_mm_s), 15.0)
        self.assertLessEqual(snapshot["last_recovery_time_s"], 4.0)


class OutlierTests(unittest.TestCase):
    def test_single_false_edge_frame_is_rejected(self):
        estimator = BallEstimator()
        estimator.update(324.0, 0.9, 0.0)
        self.assertFalse(estimator.update(74.0, 0.9, 0.05))
        self.assertAlmostEqual(estimator.position_mm, 0.0)

    def test_persistent_new_position_is_reacquired_as_disturbance(self):
        estimator = BallEstimator()
        estimator.update(324.0, 0.9, 0.0)
        self.assertFalse(estimator.update(280.0, 0.9, 0.05))
        self.assertTrue(estimator.update(280.0, 0.9, 0.10))
        self.assertGreater(estimator.position_mm, 10.0)


if __name__ == "__main__":
    unittest.main()
