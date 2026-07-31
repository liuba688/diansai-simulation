import math
import pathlib
import sys
import unittest


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from task1_timed_control import BallTaskController  # noqa: E402


class TimedTaskTests(unittest.TestCase):
    @staticmethod
    def _position_to_pixel(position_mm):
        if position_mm >= 0.0:
            return 324.0 - position_mm * (324.0 - 212.0) / 50.0
        return 324.0 + (-position_mm) * (435.0 - 324.0) / 50.0

    def test_action_table_contains_both_directions_and_final_hold(self):
        controller = BallTaskController()
        controller.state = "POS_PUSH"
        self.assertGreater(controller._command_angle(0.0, 0.0), 0.0)
        controller.state = "POS_BRAKE"
        self.assertLess(controller._command_angle(30.0, 70.0), 0.0)
        controller.state = "NEG_PUSH"
        self.assertLess(controller._command_angle(50.0, 0.0), 0.0)
        controller.state = "NEG_BRAKE"
        self.assertGreater(controller._command_angle(-20.0, -80.0), 0.0)
        controller.state = "NEG_SETTLE"
        self.assertLess(controller._command_angle(-30.0, 0.0), 0.0)

    def test_nominal_friction_plant_runs_complete_sequence(self):
        controller = BallTaskController()
        dt = 0.01
        now = 0.0
        position_mm = 0.0
        velocity_mm_s = 0.0
        beam_angle_deg = 0.0
        next_vision_time = 0.0
        visited = set()
        transitions = []
        previous_state = controller.state
        positive_position_mm = None
        snapshot = None

        while now <= 5.5:
            measured_pixel = None
            if now + 1e-9 >= next_vision_time:
                measured_pixel = self._position_to_pixel(position_mm)
                next_vision_time += 0.05
            snapshot = controller.update(measured_pixel, 0.95, now)
            visited.add(snapshot["state"])
            if snapshot["state"] != previous_state:
                transitions.append(
                    (
                        round(snapshot["elapsed_s"], 2),
                        snapshot["state"],
                        round(position_mm, 1),
                        round(velocity_mm_s, 1),
                    )
                )
                previous_state = snapshot["state"]

            if (
                snapshot["state"] == "NEG_PUSH"
                and positive_position_mm is None
            ):
                positive_position_mm = position_mm

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
                * 9.80665
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
                direction = (
                    velocity_mm_s
                    if abs(velocity_mm_s) >= 1.0
                    else gravity_acceleration_mm_s2
                )
                acceleration_mm_s2 = gravity_acceleration_mm_s2 - math.copysign(
                    35.0,
                    direction,
                )

            position_mm += (
                velocity_mm_s * dt
                + 0.5 * acceleration_mm_s2 * dt * dt
            )
            velocity_mm_s += acceleration_mm_s2 * dt

            if snapshot["state"] in ("COMPLETE", "FAULT"):
                break
            now += dt

        self.assertIsNotNone(snapshot)
        self.assertEqual(
            snapshot["state"],
            "COMPLETE",
            msg="state={} reason={} t={:.2f} x={:.1f} v={:.1f} phases={}".format(
                snapshot["state"],
                snapshot["fault_reason"],
                snapshot["elapsed_s"],
                position_mm,
                velocity_mm_s,
                transitions,
            ),
        )
        self.assertTrue(
            {
                "POS_PUSH",
                "POS_BRAKE",
                "POS_SETTLE",
                "NEG_PUSH",
                "NEG_BRAKE",
                "NEG_SETTLE",
            }.issubset(visited)
        )
        self.assertIsNotNone(positive_position_mm)
        self.assertLessEqual(abs(positive_position_mm - 50.0), 10.0)
        self.assertLessEqual(abs(position_mm + 50.0), 10.0)
        self.assertLessEqual(snapshot["elapsed_s"], 5.0)


if __name__ == "__main__":
    unittest.main()
