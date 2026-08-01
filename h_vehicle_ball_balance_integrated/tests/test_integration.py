import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIX = ROOT / "maixcam"
sys.path.insert(0, str(MAIX))

from ball_estimator import BallEstimator, pixel_to_mm
from mission_protocol import (
    BALL_STATE, MODE_SELECT, Parser, build_ball, build_packet,
    crc8, decode_command,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc8(bytes((2, MODE_SELECT, 0, 7))), 0xD8)

    def test_chunked_command_decode(self):
        payload = bytes((4, 9, 0x0C, 0xFE, 2, 0xA5, 0x78, 0x56, 0x34, 0x12))
        frame = build_packet(MODE_SELECT, 17, payload)
        parser = Parser()
        packets = []
        for byte in frame:
            packets.extend(parser.feed(bytes((byte,))))
        self.assertEqual(len(packets), 1)
        command = decode_command(packets[0])
        self.assertEqual(command["task_id"], 4)
        self.assertEqual(command["run_id"], 9)
        self.assertEqual(command["target_x10_mm"], -500)
        self.assertEqual(command["speed_tier"], 2)
        self.assertEqual(command["timestamp_ms"], 0x12345678)

    def test_ball_payload_matches_mcu_layout(self):
        frame = build_ball(3, 5, 12, 3, -487, -61, 230, 14, 123456)
        packet = Parser().feed(frame)[0]
        self.assertEqual(packet["type"], BALL_STATE)
        self.assertEqual(len(packet["payload"]), 14)
        self.assertEqual(packet["payload"][:3], bytes((5, 12, 3)))
        self.assertEqual(packet["payload"][3:5], bytes((0x19, 0xFE)))

    def test_crc_rejection(self):
        frame = bytearray(build_packet(MODE_SELECT, 1, bytes(10)))
        frame[-1] ^= 0x80
        parser = Parser()
        self.assertEqual(parser.feed(frame), [])
        self.assertEqual(parser.crc_errors, 1)


class EstimatorTests(unittest.TestCase):
    def test_three_point_calibration(self):
        self.assertAlmostEqual(pixel_to_mm(212), 50.0)
        self.assertAlmostEqual(pixel_to_mm(324), 0.0)
        self.assertAlmostEqual(pixel_to_mm(435), -50.0)

    def test_single_frame_outlier_is_rejected(self):
        estimator = BallEstimator()
        self.assertTrue(estimator.update(324, 0.9, 1.0))
        self.assertFalse(estimator.update(212, 0.9, 1.02))
        position, _, _, valid = estimator.estimate(1.02)
        self.assertTrue(valid)
        self.assertAlmostEqual(position, 0.0)


class SourceContractTests(unittest.TestCase):
    def test_maixcam_has_no_direct_x42s_driver(self):
        names = {path.name for path in MAIX.iterdir() if path.is_file()}
        self.assertNotIn("x42s_uart.py", names)
        source = (MAIX / "main.py").read_text(encoding="utf-8")
        self.assertNotIn("build_position_frame", source)

    def test_six_tasks_and_emm_driver_are_present(self):
        menu = (ROOT / "mspm0/project/code/car_menu.h").read_text(encoding="utf-8")
        for task_id in range(1, 7):
            self.assertIn("CAR_TASK_{}".format(task_id), menu)
        driver = (ROOT / "mspm0/project/code/zdt_emm_v5.c").read_text(encoding="utf-8")
        self.assertIn("0xFDU", driver)
        self.assertIn("0xFEU", driver)
        self.assertNotIn("0xFCU", driver)

    def test_task3_is_completion_priority(self):
        source = (ROOT / "mspm0/project/code/ball_balance.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("tick - c->start_tick >= 500U", source)
        self.assertIn("TASK3_VISION_GRACE_TICKS", source)
        self.assertIn("task3_completion_priority_v7", source)

    def test_x42s_uses_one_power_on_zero_and_returns_home(self):
        driver = (ROOT / "mspm0/project/code/zdt_emm_v5.c").read_text(
            encoding="utf-8"
        )
        app = (ROOT / "mspm0/project/code/car_app.c").read_text(
            encoding="utf-8"
        )
        mission = (ROOT / "mspm0/project/code/h_mission.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("zdt_emm_begin(0U)", app)
        self.assertIn("ZDT_EMM_READY == state", driver)
        self.assertIn("zdt_emm_return_home", driver)
        self.assertGreaterEqual(mission.count("zdt_emm_return_home"), 2)

    def test_curve_fusion_and_forward_only_soft_stop_contract(self):
        code = ROOT / "mspm0/project/code"
        app = (code / "car_app.c").read_text(encoding="utf-8")
        follow = (code / "line_follow.c").read_text(encoding="utf-8")
        follow_header = (code / "line_follow.h").read_text(encoding="utf-8")
        menu = (code / "car_menu.c").read_text(encoding="utf-8")

        self.assertIn("LINE_FOLLOW_ARC_FUSION_ENABLE         (1)", follow_header)
        self.assertIn("LINE_FOLLOW_ERROR_FILTER_ALPHA", follow)
        self.assertIn("LINE_FOLLOW_CURVE_TRIGGER_ERROR", follow)
        self.assertIn("TRACK_PHASE_ADVANCE_CM", follow)
        self.assertIn("odometry_entry_ready", follow)
        self.assertIn("!follow->curve_active && !follow->curve_aborted", follow)
        self.assertIn(
            "TRACK_CURVE_ODOM_ENTRY_LEAD_CM     (7.0f)", follow_header
        )
        self.assertIn("TRACK_CURVE_2_EXTRA_ADVANCE_CM", follow)
        self.assertIn(
            "TRACK_CURVE_2_EXTRA_ADVANCE_CM     (5.0f)", follow_header
        )
        self.assertIn("curve_blend", follow)
        self.assertNotIn("curve_boost", follow)
        self.assertIn("LINE_FOLLOW_CURVE_LOADED_BLEND_STEP", follow_header)
        self.assertIn("LINE_FOLLOW_CURVE_REACQUIRE_TICKS", follow_header)
        self.assertIn("LINE_FOLLOW_CURVE_LOST_TURN_RIGHT_RPM", follow)
        self.assertIn("LINE_FOLLOW_CURVE_LOST_FLAT_RIGHT_RPM", follow)
        self.assertIn("LINE_FOLLOW_CURVE_MIN_FORWARD_RPM", app)
        self.assertIn("0.24f, 12.0f", menu)

        self.assertIn("CAR_FINISH_STOP_RAMP", app)
        self.assertIn("CAR_FINISH_FALLBACK_DISTANCE_CM", app)
        self.assertIn("CAR_FINISH_FALLBACK_MIN_SENSORS", app)
        self.assertIn("@RUNLOG,DATA,4", app)
        self.assertIn("CAR_LAP_DECEL_START_CM", app)
        self.assertIn("CAR_LAP_DECEL_START_CM                  (525.0f)", app)
        self.assertIn("CAR_LAP_DECEL_END_CM                    (565.0f)", app)
        self.assertIn("line_follow.params.max_curve_rpm", app)
        self.assertIn("CAR_AB_DECEL_START_CM", app)
        self.assertIn("car_update_soft_stop", app)
        self.assertNotIn("CAR_FINISH_REVERSE_RPM", app)
        self.assertNotIn("CAR_FINISH_BACKUP", app)
        self.assertNotIn("CAR_FINISH_FORWARD_SEEK", app)


if __name__ == "__main__":
    unittest.main()
