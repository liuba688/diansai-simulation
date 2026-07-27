"""MaixCAM UART adapter for the ball-car protocol."""

import config
from vision_protocol import (
    PacketParser,
    build_target_packet,
    decode_car_status,
)


class VisionUartLink:
    def __init__(self):
        self._serial = None
        self._sequence = 0
        self._parser = PacketParser()
        self.last_status = None

        if not config.UART_ENABLED:
            print("Vision UART disabled in config.py")
            return

        try:
            from maix import pinmap, uart

            pinmap.set_pin_function(
                config.UART_TX_PIN,
                config.UART_TX_FUNCTION,
            )
            pinmap.set_pin_function(
                config.UART_RX_PIN,
                config.UART_RX_FUNCTION,
            )
            self._serial = uart.UART(
                config.UART_DEVICE,
                config.UART_BAUDRATE,
            )
            print(
                "Vision UART ready: {} @ {} TX={} RX={}".format(
                    config.UART_DEVICE,
                    config.UART_BAUDRATE,
                    config.UART_TX_PIN,
                    config.UART_RX_PIN,
                )
            )
        except Exception as error:
            self._serial = None
            print("Vision UART init failed: {}".format(error))

    @property
    def ready(self):
        return self._serial is not None

    def send_target(
        self,
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
        if self._serial is None:
            return False

        packet = build_target_packet(
            self._sequence,
            flags,
            center_x,
            center_y,
            width,
            height,
            frame_width,
            frame_height,
            confidence,
            candidate_count,
        )
        try:
            self._serial.write(packet)
            self._sequence = (self._sequence + 1) & 0xFF
            return True
        except Exception as error:
            print("Vision UART write failed: {}".format(error))
            return False

    def poll_status(self):
        if self._serial is None:
            return self.last_status

        try:
            data = self._serial.read()
        except Exception as error:
            print("Vision UART read failed: {}".format(error))
            return self.last_status

        if not data:
            return self.last_status

        for byte in data:
            packet = self._parser.feed(byte)
            if packet is None:
                continue
            status = decode_car_status(packet)
            if status is not None:
                self.last_status = status
        return self.last_status

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
