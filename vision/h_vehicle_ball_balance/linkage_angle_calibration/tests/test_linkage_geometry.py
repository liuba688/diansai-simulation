import math
import pathlib
import sys
import unittest


MODULE_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from linkage_geometry import (  # noqa: E402
    motor_deg_from_pulses,
    theoretical_beam_deg_from_pulses,
)


class LinkageGeometryTests(unittest.TestCase):
    def test_known_motor_mapping(self):
        self.assertTrue(math.isclose(motor_deg_from_pulses(160), 18.0))

    def test_zero_and_symmetry(self):
        self.assertEqual(theoretical_beam_deg_from_pulses(0), 0.0)
        self.assertTrue(
            math.isclose(
                theoretical_beam_deg_from_pulses(40),
                -theoretical_beam_deg_from_pulses(-40),
                rel_tol=1e-12,
            )
        )

    def test_small_calibration_targets_are_conservative(self):
        angle_40 = theoretical_beam_deg_from_pulses(40)
        angle_80 = theoretical_beam_deg_from_pulses(80)
        self.assertGreater(angle_40, 0.60)
        self.assertLess(angle_40, 0.65)
        self.assertGreater(angle_80, 1.20)
        self.assertLess(angle_80, 1.30)


if __name__ == "__main__":
    unittest.main()
