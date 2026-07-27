"""Host-side tests for the MaixCAM/MSPM0 UART wire format."""

import pathlib
import struct
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "maixcam_ball_car"))

from vision_protocol import (  # noqa: E402
    HEADER_0,
    HEADER_1,
    PACKET_TYPE_BALL_TARGET,
    PACKET_TYPE_CAR_STATUS,
    PROTOCOL_VERSION,
    STATUS_FLAG_ENABLED,
    STATUS_FLAG_MAGNET_ON,
    TARGET_FLAG_CONFIRMED,
    TARGET_FLAG_VALID,
    PacketParser,
    build_target_packet,
    crc8,
    decode_car_status,
)


class VisionProtocolTests(unittest.TestCase):
    def test_target_packet_layout_and_crc(self):
        packet = build_target_packet(
            sequence=0x2A,
            flags=TARGET_FLAG_VALID | TARGET_FLAG_CONFIRMED,
            center_x=160,
            center_y=200,
            width=40,
            height=42,
            frame_width=320,
            frame_height=320,
            confidence=78,
            candidate_count=2,
        )

        self.assertEqual(len(packet), 23)
        self.assertEqual(packet[:2], bytes((HEADER_0, HEADER_1)))
        self.assertEqual(packet[2], PROTOCOL_VERSION)
        self.assertEqual(packet[3], PACKET_TYPE_BALL_TARGET)
        self.assertEqual(packet[4], 16)
        self.assertEqual(packet[5], 0x2A)
        self.assertEqual(packet[-1], crc8(packet[2:-1]))

        payload = struct.unpack("<BHHHHHHBBB", packet[6:-1])
        self.assertEqual(payload[0], 0x03)
        self.assertEqual(payload[1:7], (160, 200, 40, 42, 320, 320))
        self.assertEqual(payload[7:10], (78, 2, 0))

    def test_parser_resynchronizes_after_noise_and_bad_crc(self):
        packet = build_target_packet(
            7,
            TARGET_FLAG_VALID,
            100,
            120,
            30,
            31,
            320,
            320,
            75,
            1,
        )
        damaged = packet[:-1] + bytes((packet[-1] ^ 0xFF,))
        parser = PacketParser()
        decoded = []

        for byte in b"\x00\x7f\xaa\x01" + damaged + b"\x99" + packet:
            item = parser.feed(byte)
            if item is not None:
                decoded.append(item)

        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0]["sequence"], 7)
        self.assertEqual(decoded[0]["type"], PACKET_TYPE_BALL_TARGET)

    def test_decode_car_status(self):
        payload = struct.pack(
            "<BBBBhh",
            4,
            STATUS_FLAG_ENABLED | STATUS_FLAG_MAGNET_ON,
            0,
            0x18,
            123,
            -45,
        )
        header = bytes(
            (
                HEADER_0,
                HEADER_1,
                PROTOCOL_VERSION,
                PACKET_TYPE_CAR_STATUS,
                len(payload),
                9,
            )
        )
        raw = header + payload
        raw += bytes((crc8(raw[2:]),))

        parser = PacketParser()
        packet = None
        for byte in raw:
            packet = parser.feed(byte) or packet

        status = decode_car_status(packet)
        self.assertEqual(status["sequence"], 9)
        self.assertEqual(status["state"], 4)
        self.assertEqual(status["line_mask"], 0x18)
        self.assertAlmostEqual(status["left_rpm"], 12.3)
        self.assertAlmostEqual(status["right_rpm"], -4.5)


if __name__ == "__main__":
    unittest.main()
