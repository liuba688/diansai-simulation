"""MaixCAM 到 MSPM0 的 17 字节视觉数据包。"""

import struct

import config


HEADER_0 = 0xAA
HEADER_1 = 0x55
PAYLOAD_LENGTH = 11


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def build_target_packet(
    sequence, status, target_x, target_y, laser_x, laser_y, confidence
):
    packet = struct.pack(
        "<BBBBBBBhhhhB",
        HEADER_0,
        HEADER_1,
        config.PROTOCOL_VERSION,
        config.PACKET_TYPE_TARGET,
        PAYLOAD_LENGTH,
        sequence & 0xFF,
        status & 0xFF,
        int(target_x),
        int(target_y),
        int(laser_x),
        int(laser_y),
        max(0, min(100, int(confidence))),
    )
    return packet + bytes([crc8(packet[2:])])


class VisionUartLink:
    def __init__(self):
        self._serial = None
        self._sequence = 0
        if not config.UART_ENABLED:
            print("UART disabled in config.py")
            return
        try:
            from maix import pinmap, uart

            pinmap.set_pin_function(config.UART_TX_PIN, "UART1_TX")
            pinmap.set_pin_function(config.UART_RX_PIN, "UART1_RX")
            self._serial = uart.UART(config.UART_DEVICE, config.UART_BAUDRATE)
            print("UART ready: {} @ {}".format(config.UART_DEVICE, config.UART_BAUDRATE))
        except Exception as error:
            print("UART init failed: {}".format(error))

    def send_track(self, track):
        if self._serial is None:
            return
        target_x = track.center_x if track.valid else 0
        target_y = track.center_y if track.valid else 0
        self._serial.write(
            build_target_packet(
                self._sequence,
                track.status,
                target_x,
                target_y,
                config.AIM_REFERENCE_X,
                config.AIM_REFERENCE_Y,
                track.confidence,
            )
        )
        self._sequence = (self._sequence + 1) & 0xFF

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
