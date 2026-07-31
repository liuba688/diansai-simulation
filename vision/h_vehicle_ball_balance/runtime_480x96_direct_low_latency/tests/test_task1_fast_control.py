import math
import pathlib
import sys
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from task1_fast_control import (  # noqa: E402
    BallTaskController,
    CENTER_HOLD_TIME_S,
    FINAL_SETTLE_TIME_S,
    GRAVITY_M_S2,
    MAX_BEAM_ANGLE_DEG,
    NEGATIVE_MOVE_TIME_S,
    POSITIVE_MOVE_TIME_S,
    POSITIVE_SETTLE_TIME_S,
    ROLLING_ACCELERATION_FACTOR,
    acceleration_to_beam_angle_deg,
    quintic_reference,
)


class QuinticReferenceTests(unittest.TestCase):
    def test_reference_has_zero_velocity_and_acceleration_at_ends(self):
        start = quintic_reference(0.0, 50.0, 1.0, 0.0)
        end = quintic_reference(0.0, 50.0, 1.0, 1.0)
        self.assertEqual(start, (0.0, 0.0, 0.0))
        self.assertEqual(end, (50.0, 0.0, 0.0))

    def test_reference_midpoint_is_symmetric(self):
        position, velocity, acceleration = quintic_reference(
            0.0,
            50.0,
            1.0,
            0.5,
        )
        self.assertAlmostEqual(position, 25.0)
        self.assertGreater(velocity, 90.0)
        self.assertAlmostEqual(acceleration, 0.0)

    def test_planned_acceleration_fits_calibrated_three_degree_range(self):
        for start, end, duration in (
            (0.0, 50.0, POSITIVE_MOVE_TIME_S),
            (50.0, -50.0, NEGATIVE_MOVE_TIME_S),
        ):
            for index in range(101):
                elapsed = duration * index / 100.0
                _, _, acceleration = quintic_reference(
                    start,
                    end,
                    duration,
                    elapsed,
                )
                self.assertLessEqual(
                    abs(acceleration_to_beam_angle_deg(acceleration)),
                    MAX_BEAM_ANGLE_DEG,
                )


class FastTaskStateTests(unittest.TestCase):
    def test_ideal_task_completes_inside_five_seconds(self):
        controller = BallTaskController()
        controller.state = "HOLD_CENTER"
        controller.start_time = 0.0
        controller.state_since = 0.0

        positive_start = CENTER_HOLD_TIME_S + 0.05
        controller._update_state(0.0, 0.0, True, positive_start)
        self.assertEqual(controller.state, "MOVE_POSITIVE")

        positive_end = positive_start + POSITIVE_MOVE_TIME_S + 0.01
        controller._update_state(50.0, 0.0, True, positive_end)
        self.assertEqual(controller.state, "HOLD_POSITIVE")
        controller._update_state(50.0, 0.0, True, positive_end + 0.01)
        negative_start = (
            positive_end + POSITIVE_SETTLE_TIME_S + 0.02
        )
        controller._update_state(50.0, 0.0, True, negative_start)
        self.assertEqual(controller.state, "MOVE_NEGATIVE")

        negative_end = negative_start + NEGATIVE_MOVE_TIME_S + 0.01
        controller._update_state(-50.0, 0.0, True, negative_end)
        self.assertEqual(controller.state, "HOLD_NEGATIVE")
        controller._update_state(-50.0, 0.0, True, negative_end + 0.01)
        complete_time = negative_end + FINAL_SETTLE_TIME_S + 0.02
        controller._update_state(-50.0, 0.0, True, complete_time)
        self.assertEqual(controller.state, "COMPLETE")
        self.assertLess(complete_time - controller.start_time, 5.0)

    def test_task_timeout_is_a_fault(self):
        controller = BallTaskController()
        controller.state = "MOVE_NEGATIVE"
        controller.start_time = 0.0
        controller._update_state(0.0, 0.0, True, 5.01)
        self.assertEqual(controller.state, "FAULT")
        self.assertEqual(controller.fault_reason, "task timeout")

    def test_positive_and_negative_stiction_thresholds_are_independent(self):
        controller = BallTaskController()
        controller.state = "MOVE_POSITIVE"
        controller.motion_direction = 1.0
        controller.target_goal_mm = 50.0
        angle = 0.2
        for _ in range(3):
            angle = controller._apply_stiction_compensation(
                angle,
                position_mm=20.0,
                velocity_mm_s=0.0,
                dt=0.05,
            )
        self.assertGreaterEqual(angle, 1.05)

        controller.stiction_active = False
        controller.stiction_stationary_time_s = 0.0
        controller.state = "MOVE_NEGATIVE"
        controller.motion_direction = -1.0
        controller.target_goal_mm = -50.0
        angle = -0.2
        for _ in range(3):
            angle = controller._apply_stiction_compensation(
                angle,
                position_mm=20.0,
                velocity_mm_s=0.0,
                dt=0.05,
            )
        self.assertLessEqual(angle, -1.15)


class NominalPlantSimulationTests(unittest.TestCase):
    @staticmethod
    def _position_to_pixel(position_mm):
        if position_mm >= 0.0:
            return 324.0 - position_mm * (324.0 - 212.0) / 50.0
        return 324.0 + (-position_mm) * (435.0 - 324.0) / 50.0

    def test_nominal_delayed_friction_plant_completes_inside_five_seconds(self):
        controller = BallTaskController()
        dt = 0.01
        now = 0.0
        position_mm = 0.0
        velocity_mm_s = 0.0
        beam_angle_deg = 0.0
        next_vision_time = 0.0
        snapshot = None
        positive_arrival_error_mm = None
        previous_state = controller.state

        while now <= 5.5:
            measured_pixel = None
            if now + 1e-9 >= next_vision_time:
                measured_pixel = self._position_to_pixel(position_mm)
                next_vision_time += 0.05

            snapshot = controller.update(measured_pixel, 0.95, now)
            requested_angle_deg = snapshot["requested_beam_angle_deg"]

            # Approximate the linkage/X42S response with a first-order lag and
            # a conservative 15 deg/s beam-angle rate limit.
            requested_rate_deg_s = (
                requested_angle_deg - beam_angle_deg
            ) / 0.07
            requested_rate_deg_s = max(
                -15.0,
                min(15.0, requested_rate_deg_s),
            )
            beam_angle_deg += requested_rate_deg_s * dt

            gravity_acceleration_mm_s2 = (
                ROLLING_ACCELERATION_FACTOR
                * GRAVITY_M_S2
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
                rolling_friction_mm_s2 = math.copysign(
                    35.0,
                    friction_direction,
                )
                acceleration_mm_s2 = (
                    gravity_acceleration_mm_s2
                    - rolling_friction_mm_s2
                )

            position_mm += (
                velocity_mm_s * dt
                + 0.5 * acceleration_mm_s2 * dt * dt
            )
            velocity_mm_s += acceleration_mm_s2 * dt

            if (
                previous_state == "HOLD_POSITIVE"
                and snapshot["state"] == "MOVE_NEGATIVE"
            ):
                positive_arrival_error_mm = abs(position_mm - 50.0)
            previous_state = snapshot["state"]

            if snapshot["state"] in ("COMPLETE", "FAULT"):
                break
            now += dt

        self.assertIsNotNone(snapshot)
        self.assertEqual(
            snapshot["state"],
            "COMPLETE",
            msg="state={} reason={} t={:.2f} x={:.1f} v={:.1f}".format(
                snapshot["state"],
                snapshot["fault_reason"],
                snapshot["elapsed_s"],
                position_mm,
                velocity_mm_s,
            ),
        )
        self.assertLessEqual(snapshot["elapsed_s"], 5.0)
        self.assertIsNotNone(positive_arrival_error_mm)
        self.assertLessEqual(positive_arrival_error_mm, 10.0)
        self.assertLessEqual(abs(position_mm + 50.0), 10.0)


if __name__ == "__main__":
    unittest.main()
