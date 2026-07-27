"""Binary UART protocol shared by MaixCAM and the MSPM0 car."""

import struct


HEADER_0 = 0xAA
HEADER_1 = 0x55
PROTOCOL_VERSION = 1

PACKET_TYPE_BALL_TARGET = 0x10
PACKET_TYPE_CAR_STATUS = 0x20

TARGET_PAYLOAD_LENGTH = 16
STATUS_PAYLOAD_LENGTH = 8
MAX_PAYLOAD_LENGTH = 16

TARGET_FLAG_VALID = 1 << 0
TARGET_FLAG_CONFIRMED = 1 << 1
TARGET_FLAG_CLOSE = 1 << 2
TARGET_FLAG_MULTIPLE = 1 << 3

STATUS_FLAG_ENABLED = 1 << 0
STATUS_FLAG_MAGNET_ON = 1 << 1
STATUS_FLAG_PAYLOAD_HELD = 1 << 2
STATUS_FLAG_FAULT = 1 << 3


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            if value & 0x80:
                value = ((value << 1) ^ 0x07) & 0xFF
            else:
                value = (value << 1) & 0xFF
    return value


def _clamp_u16(value):
    return max(0, min(65535, int(value)))


def _clamp_u8(value):
    return max(0, min(255, int(value)))


def build_target_packet(
    sequence,
    flags,
    center_x,
    center_y,
    width,
    height,
    frame_width,
    frame_height,
    confidence,
    candidate_count,
):
    payload = struct.pack(
        "<BHHHHHHBBB",
        _clamp_u8(flags),
        _clamp_u16(center_x),
        _clamp_u16(center_y),
        _clamp_u16(width),
        _clamp_u16(height),
        _clamp_u16(frame_width),
        _clamp_u16(frame_height),
        max(0, min(100, int(round(confidence)))),
        _clamp_u8(candidate_count),
        0,
    )
    header = bytes(
        (
            HEADER_0,
            HEADER_1,
            PROTOCOL_VERSION,
            PACKET_TYPE_BALL_TARGET,
            len(payload),
            sequence & 0xFF,
        )
    )
    packet = header + payload
    return packet + bytes((crc8(packet[2:]),))


def decode_car_status(packet):
    if packet["version"] != PROTOCOL_VERSION:
        return None
    if packet["type"] != PACKET_TYPE_CAR_STATUS:
        return None
    payload = packet["payload"]
    if len(payload) != STATUS_PAYLOAD_LENGTH:
        return None

    state, flags, fault, line_mask, left_x10, right_x10 = struct.unpack(
        "<BBBBhh",
        payload,
    )
    return {
        "sequence": packet["sequence"],
        "state": state,
        "flags": flags,
        "fault": fault,
        "line_mask": line_mask,
        "left_rpm": left_x10 / 10.0,
        "right_rpm": right_x10 / 10.0,
    }


class PacketParser:
    WAIT_HEADER_0 = 0
    WAIT_HEADER_1 = 1
    READ_VERSION = 2
    READ_TYPE = 3
    READ_LENGTH = 4
    READ_SEQUENCE = 5
    READ_PAYLOAD = 6
    READ_CRC = 7

    def __init__(self):
        self.reset()

    def reset(self, current_byte=None):
        self.state = (
            self.WAIT_HEADER_1
            if current_byte == HEADER_0
            else self.WAIT_HEADER_0
        )
        self.version = 0
        self.packet_type = 0
        self.payload_length = 0
        self.sequence = 0
        self.payload = bytearray()
        self.crc_data = bytearray()

    def feed(self, byte):
        byte = int(byte) & 0xFF

        if self.state == self.WAIT_HEADER_0:
            if byte == HEADER_0:
                self.state = self.WAIT_HEADER_1
            return None

        if self.state == self.WAIT_HEADER_1:
            if byte == HEADER_1:
                self.state = self.READ_VERSION
                self.crc_data = bytearray()
            elif byte != HEADER_0:
                self.state = self.WAIT_HEADER_0
            return None

        if self.state == self.READ_VERSION:
            self.version = byte
            self.crc_data.append(byte)
            self.state = self.READ_TYPE
            return None

        if self.state == self.READ_TYPE:
            self.packet_type = byte
            self.crc_data.append(byte)
            self.state = self.READ_LENGTH
            return None

        if self.state == self.READ_LENGTH:
            self.payload_length = byte
            self.crc_data.append(byte)
            if byte > MAX_PAYLOAD_LENGTH:
                self.reset(byte)
                return None
            self.state = self.READ_SEQUENCE
            return None

        if self.state == self.READ_SEQUENCE:
            self.sequence = byte
            self.crc_data.append(byte)
            self.payload = bytearray()
            self.state = (
                self.READ_CRC
                if self.payload_length == 0
                else self.READ_PAYLOAD
            )
            return None

        if self.state == self.READ_PAYLOAD:
            self.payload.append(byte)
            self.crc_data.append(byte)
            if len(self.payload) >= self.payload_length:
                self.state = self.READ_CRC
            return None

        if self.state == self.READ_CRC:
            if byte != crc8(self.crc_data):
                self.reset(byte)
                return None

            packet = {
                "version": self.version,
                "type": self.packet_type,
                "sequence": self.sequence,
                "payload": bytes(self.payload),
            }
            self.reset()
            return packet

        self.reset()
        return None
