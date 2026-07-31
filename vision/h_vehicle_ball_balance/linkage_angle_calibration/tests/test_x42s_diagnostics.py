import pathlib
import sys
import unittest


MODULE_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from x42s_uart import (  # noqa: E402
    X42SMotor,
    build_current_position_query_frame,
    build_stop_frame,
)


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.rx = bytearray()

    def write(self, frame):
        frame = bytes(frame)
        self.writes.append(frame)
        if frame[1] == 0x36:
            # +90 degrees = 16384 encoder units.
            self.rx.extend([0x01, 0x36, 0x00, 0x00, 0x00, 0x40, 0x00, 0x6B])
        else:
            self.rx.extend([0x01, frame[1], 0x02, 0x6B])
        return len(frame)

    def read(self):
        data = bytes(self.rx)
        self.rx.clear()
        return data


class X42DiagnosticsTests(unittest.TestCase):
    def test_position_query_frame(self):
        self.assertEqual(
            build_current_position_query_frame(),
            bytes([0x01, 0x36, 0x6B]),
        )

    def test_read_current_position(self):
        serial = FakeSerial()
        motor = X42SMotor(serial_port=serial)
        motor.initialize()
        self.assertAlmostEqual(motor.read_current_position_deg(), 90.0)

    def test_emergency_stop_latches_driver(self):
        serial = FakeSerial()
        motor = X42SMotor(serial_port=serial)
        motor.initialize()
        self.assertTrue(motor.emergency_stop())
        self.assertEqual(serial.writes[-1], build_stop_frame())
        self.assertFalse(motor.initialized)
        self.assertFalse(motor.request_target(20, now=1.0))


if __name__ == "__main__":
    unittest.main()
