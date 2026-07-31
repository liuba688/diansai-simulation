import pathlib
import sys
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from center_control import (  # noqa: E402
    BallEstimator,
    BallTaskController,
    beam_angle_to_motor_pulses,
)


class LinkageMapTests(unittest.TestCase):
    def test_measured_piecewise_points(self):
        self.assertEqual(beam_angle_to_motor_pulses(0.0), 0)
        self.assertEqual(beam_angle_to_motor_pulses(1.0), 80)
        self.assertEqual(beam_angle_to_motor_pulses(-1.0), -80)
        self.assertEqual(beam_angle_to_motor_pulses(3.0), 160)

    def test_fast_profile_uses_one_point_eight_deg_limit(self):
        self.assertEqual(beam_angle_to_motor_pulses(1.5), 100)
        controller = BallTaskController()
        self.assertEqual(controller.active_drive_limit_pulses, 112)


class CenterControllerTests(unittest.TestCase):
    def _arm(self, controller):
        now = 0.0
        for _ in range(7):
            controller.update(324.0, 0.9, now)
            now += 0.1
        return now

    def test_waits_then_holds_center(self):
        controller = BallTaskController()
        self._arm(controller)
        self.assertEqual(controller.state, "HOLD_CENTER")

    def test_pixel_calibration_mode_never_drives_motor(self):
        controller = BallTaskController(pixel_calibration_only=True)
        for index, pixel_x in enumerate((212.0, 324.0, 435.0)):
            snapshot = controller.update(pixel_x, 0.9, index * 0.5)
            self.assertEqual(snapshot["state"], "PIXEL_CALIBRATION")
            self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_positive_position_commands_negative_beam(self):
        controller = BallTaskController()
        now = self._arm(controller)
        snapshot = controller.update(300.0, 0.9, now)
        self.assertLess(snapshot["requested_beam_angle_deg"], 0.0)
        self.assertLess(snapshot["motor_target_pulses"], 0)

    def test_vision_timeout_faults_and_levels(self):
        controller = BallTaskController()
        now = self._arm(controller)
        snapshot = controller.update(None, 0.0, now + 0.2)
        self.assertEqual(snapshot["state"], "FAULT")
        self.assertEqual(snapshot["motor_target_pulses"], 0)

    def test_center_hold_starts_positive_reference(self):
        controller = BallTaskController()
        now = self._arm(controller)
        controller.update(324.0, 0.9, now + 1.1)
        snapshot = controller.update(324.0, 0.9, now + 1.2)
        self.assertEqual(snapshot["state"], "TO_POSITIVE")
        self.assertGreater(snapshot["target_mm"], 0.0)
        self.assertGreater(snapshot["motor_target_pulses"], 0)

    def test_explicit_stiction_boost_replaces_slow_moving_integral(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.state_since = 0.0
        controller.target_goal_mm = 50.0
        controller.target_mm = 50.0

        snapshot = None
        for index in range(4):
            snapshot = controller.update(250.0, 0.9, index * 0.1)

        self.assertTrue(snapshot["stiction_active"])
        self.assertEqual(snapshot["stiction_event_count"], 1)
        self.assertEqual(snapshot["integral_angle_deg"], 0.0)
        self.assertGreaterEqual(snapshot["requested_beam_angle_deg"], 1.05)

    def test_stiction_boost_releases_when_ball_is_moving(self):
        controller = BallTaskController()
        controller.state = "TO_POSITIVE"
        controller.start_time = 0.0
        controller.state_since = 0.0
        controller.target_goal_mm = 50.0
        controller.target_mm = 50.0

        for index in range(4):
            controller.update(250.0, 0.9, index * 0.1)
        snapshot = controller.update(240.0, 0.9, 0.4)

        self.assertFalse(snapshot["stiction_active"])
        self.assertGreater(snapshot["velocity_mm_s"], 12.0)

    def test_positive_hold_is_terminal_in_probe_version(self):
        controller = BallTaskController()
        controller.state = "HOLD_POSITIVE"
        controller.start_time = 0.0
        controller.state_since = 1.0
        controller.target_mm = 50.0
        controller.target_goal_mm = 50.0
        snapshot = controller.update(212.0, 0.9, 2.0)
        self.assertEqual(snapshot["state"], "HOLD_POSITIVE")
        self.assertEqual(snapshot["target_goal_mm"], 50.0)
        self.assertEqual(snapshot["target_mm"], 50.0)


class VisionOutlierTests(unittest.TestCase):
    def test_single_false_edge_detection_is_rejected(self):
        estimator = BallEstimator()
        self.assertTrue(estimator.update(324.0, 0.9, 0.0))
        self.assertFalse(estimator.update(74.0, 0.9, 0.05))
        self.assertAlmostEqual(estimator.position_mm, 0.0)
        self.assertEqual(estimator.rejected_measurement_count, 1)

        self.assertTrue(estimator.update(324.0, 0.9, 0.10))
        self.assertAlmostEqual(estimator.position_mm, 0.0)

    def test_two_consistent_frames_allow_reacquisition(self):
        estimator = BallEstimator()
        estimator.update(324.0, 0.9, 0.0)
        self.assertFalse(estimator.update(74.0, 0.9, 0.05))
        self.assertTrue(estimator.update(73.0, 0.9, 0.10))
        self.assertGreater(estimator.position_mm, 105.0)
        self.assertEqual(estimator.velocity_mm_s, 0.0)


if __name__ == "__main__":
    unittest.main()
