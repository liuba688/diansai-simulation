"""Host-side tests for MaixCAM temporal target filtering."""

import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "maixcam_ball_car"))

import config  # noqa: E402
from ball_tracker import BallTracker  # noqa: E402


class Detection:
    def __init__(self, x, y, width, height, score):
        self.x = x
        self.y = y
        self.w = width
        self.h = height
        self.score = score


def candidate(detection):
    value = BallTracker.candidate_from_detection(detection)
    if value is None:
        raise AssertionError("expected a valid candidate")
    return value


class BallTrackerTests(unittest.TestCase):
    def test_rejects_low_confidence_small_and_elongated_boxes(self):
        self.assertIsNone(
            BallTracker.candidate_from_detection(
                Detection(10, 10, 20, 20, config.CANDIDATE_CONFIDENCE_THRESHOLD - 0.01)
            )
        )
        self.assertIsNone(
            BallTracker.candidate_from_detection(
                Detection(10, 10, config.MIN_BOX_SIDE - 1, 20, 0.95)
            )
        )
        self.assertIsNone(
            BallTracker.candidate_from_detection(
                Detection(10, 10, 40, 10, 0.95)
            )
        )

    def test_requires_temporal_confirmation_before_control(self):
        tracker = BallTracker(320, 320)
        detection = Detection(140, 180, 40, 42, 0.85)

        for frame in range(1, config.CONTROL_CONFIRM_FRAMES + 1):
            displayed, controllable, selected = tracker.update(
                [candidate(detection)]
            )
            if frame < config.TRACK_CONFIRM_FRAMES:
                self.assertEqual(displayed, [])
            else:
                self.assertEqual(len(displayed), 1)

            if frame < config.CONTROL_CONFIRM_FRAMES:
                self.assertEqual(controllable, [])
                self.assertIsNone(selected)
            else:
                self.assertEqual(len(controllable), 1)
                self.assertIsNotNone(selected)

    def test_locks_selected_track_and_expires_after_misses(self):
        tracker = BallTracker(320, 320)
        target_a = Detection(140, 200, 40, 40, 0.76)
        target_b = Detection(20, 40, 30, 30, 0.96)

        selected = None
        for _ in range(config.CONTROL_CONFIRM_FRAMES):
            _, controllable, selected = tracker.update(
                [candidate(target_a), candidate(target_b)]
            )
        self.assertEqual(len(controllable), 2)
        self.assertIsNotNone(selected)
        selected_id = selected["id"]

        target_b.score = 0.99
        _, _, selected = tracker.update(
            [candidate(target_a), candidate(target_b)]
        )
        self.assertEqual(selected["id"], selected_id)

        for _ in range(config.TRACK_MAX_MISSES + 1):
            displayed, controllable, selected = tracker.update([])
        self.assertEqual(displayed, [])
        self.assertEqual(controllable, [])
        self.assertIsNone(selected)
        self.assertEqual(tracker.tracks, [])

    def test_red_box_is_available_before_green_control_confirmation(self):
        tracker = BallTracker(320, 320)
        detection = Detection(140, 180, 40, 42, 0.85)

        warning = None
        for _ in range(config.TRACK_CONFIRM_FRAMES):
            displayed, controllable, selected = tracker.update(
                [candidate(detection)]
            )
            warning = tracker.select_warning(displayed)

        self.assertEqual(len(displayed), 1)
        self.assertEqual(controllable, [])
        self.assertIsNone(selected)
        self.assertIsNotNone(warning)


if __name__ == "__main__":
    unittest.main()
