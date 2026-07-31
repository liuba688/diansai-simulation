"""ZDT X42S V2.0 / Emm_V5.0 TTL UART driver for MaixCAM bench tests."""

import time


MOTOR_ADDRESS = 0x01
CHECKSUM_BYTE = 0x6B
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"

POSITION_SPEED_RPM = 30
POSITION_ACCELERATION = 100
POSITION_COMMAND_PERIOD_S = 0.040
POSITION_ACK_TIMEOUT_S = 0.120
MAX_TARGET_OFFSET_PULSES = 160
MAX_TARGET_STEP_PULSES = 24
MAX_CONSECUTIVE_ERRORS = 3

_clock = getattr(time, "monotonic", time.time)


def _clamp(value, low, high):
    return max(low, min(high, value))


def build_enable_frame(enabled=True):
    return bytes(
        [
            MOTOR_ADDRESS,
            0xF3,
            0xAB,
            0x01 if enabled else 0x00,
            0x00,
            CHECKSUM_BYTE,
        ]
    )


def build_position_frame(
    signed_pulses,
    movement_mode=0,
    speed_rpm=POSITION_SPEED_RPM,
    acceleration=POSITION_ACCELERATION,
):
    """Build the 13-byte Emm standard position-mode frame.

    signed_pulses > 0 means CCW and signed_pulses < 0 means CW.
    movement_mode 0 is relative to the previous input target.
    movement_mode 2 is relative to the current real position.
    """

    pulses = abs(int(signed_pulses))
    direction = 0x01 if signed_pulses > 0 else 0x00
    return bytes(
        [
            MOTOR_ADDRESS,
            0xFD,
            direction,
            (int(speed_rpm) >> 8) & 0xFF,
            int(speed_rpm) & 0xFF,
            int(acceleration) & 0xFF,
            (pulses >> 24) & 0xFF,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            pulses & 0xFF,
            int(movement_mode) & 0xFF,
            0x00,
            CHECKSUM_BYTE,
        ]
    )


def build_stop_frame():
    return bytes([MOTOR_ADDRESS, 0xFE, 0x98, 0x00, CHECKSUM_BYTE])


def build_current_position_query_frame():
    """Build the Emm_V5.0 real-time position query frame."""

    return bytes([MOTOR_ADDRESS, 0x36, CHECKSUM_BYTE])


class X42SUartError(RuntimeError):
    pass


class X42SMotor:
    """Non-blocking target updater with startup response validation."""

    def __init__(self, serial_port=None):
        self._serial = serial_port
        self._owns_serial = serial_port is None
        self._rx_buffer = bytearray()
        self._pending_function = None
        self._pending_since = 0.0
        self._last_send_time = -1.0
        self._target_offset_pulses = 0
        self._initialized = False
        self._healthy = False
        self._consecutive_errors = 0
        self.ack_count = 0
        self.error_count = 0
        self.completion_count = 0
        self.position_query_count = 0
        self.last_status = 0

        if self._serial is None:
            self._open_maix_uart()

    @property
    def initialized(self):
        return self._initialized

    @property
    def healthy(self):
        return self._healthy

    @property
    def target_offset_pulses(self):
        return self._target_offset_pulses

    def _open_maix_uart(self):
        from maix import err, pinmap, uart

        err.check_raise(
            pinmap.set_pin_function(UART_TX_PIN, "UART1_TX"),
            "Failed to map A19 as UART1_TX",
        )
        err.check_raise(
            pinmap.set_pin_function(UART_RX_PIN, "UART1_RX"),
            "Failed to map A18 as UART1_RX",
        )
        self._serial = uart.UART(UART_DEVICE, UART_BAUDRATE)

    def _read_uart(self):
        data = self._serial.read()
        if data:
            self._rx_buffer.extend(data)

    def _pop_response(self):
        while self._rx_buffer:
            if self._rx_buffer[0] != MOTOR_ADDRESS:
                del self._rx_buffer[0]
                continue
            if len(self._rx_buffer) < 4:
                return None
            if self._rx_buffer[3] != CHECKSUM_BYTE:
                del self._rx_buffer[0]
                continue
            frame = bytes(self._rx_buffer[:4])
            del self._rx_buffer[:4]
            return frame
        return None

    def _drain_uart(self):
        self._read_uart()
        self._rx_buffer.clear()

    def _write(self, frame):
        written = self._serial.write(frame)
        if written is not None and int(written) < len(frame):
            raise X42SUartError(
                "UART short write: {}/{}".format(written, len(frame))
            )

    def _wait_response(self, function, timeout_s):
        deadline = _clock() + timeout_s
        while _clock() < deadline:
            self._read_uart()
            while True:
                frame = self._pop_response()
                if frame is None:
                    break
                if frame[1] != function:
                    continue
                self.last_status = frame[2]
                if frame[2] == 0x02:
                    self.ack_count += 1
                    return True
                if frame[2] == 0x9F:
                    self.completion_count += 1
                    continue
                self.error_count += 1
                raise X42SUartError(
                    "X42S rejected 0x{:02X}: status=0x{:02X}".format(
                        function,
                        frame[2],
                    )
                )
            time.sleep(0.001)
        self.error_count += 1
        raise X42SUartError(
            "X42S response timeout: function=0x{:02X}".format(function)
        )

    def _transact(self, frame, function, timeout_s=0.250):
        self._drain_uart()
        self._write(frame)
        return self._wait_response(function, timeout_s)

    def initialize(self):
        """Enable the motor and anchor target zero at its real boot position."""

        self._transact(build_enable_frame(True), 0xF3)
        self._transact(
            build_position_frame(
                0,
                movement_mode=2,
                speed_rpm=POSITION_SPEED_RPM,
                acceleration=POSITION_ACCELERATION,
            ),
            0xFD,
        )
        self._target_offset_pulses = 0
        self._pending_function = None
        self._last_send_time = _clock()
        self._consecutive_errors = 0
        self._initialized = True
        self._healthy = True
        return True

    def poll(self, now=None):
        if now is None:
            now = _clock()
        self._read_uart()
        while True:
            frame = self._pop_response()
            if frame is None:
                break
            function = frame[1]
            status = frame[2]
            self.last_status = status
            if status == 0x9F:
                self.completion_count += 1
                continue
            if status == 0x02:
                self.ack_count += 1
                self._consecutive_errors = 0
            else:
                self.error_count += 1
                self._consecutive_errors += 1
            if self._pending_function == function:
                self._pending_function = None

        if (
            self._pending_function is not None
            and now - self._pending_since >= POSITION_ACK_TIMEOUT_S
        ):
            self.error_count += 1
            self._consecutive_errors += 1
            self._pending_function = None

        self._healthy = (
            self._initialized
            and self._consecutive_errors < MAX_CONSECUTIVE_ERRORS
        )
        return self._healthy

    def request_target(self, target_offset_pulses, now=None):
        """Move toward a signed software target without crossing soft limits."""

        if now is None:
            now = _clock()
        if not self._initialized or not self._healthy:
            return False

        self.poll(now)
        if not self._healthy or self._pending_function is not None:
            return False
        if now - self._last_send_time < POSITION_COMMAND_PERIOD_S:
            return False

        requested = int(
            round(
                _clamp(
                    target_offset_pulses,
                    -MAX_TARGET_OFFSET_PULSES,
                    MAX_TARGET_OFFSET_PULSES,
                )
            )
        )
        step = int(
            _clamp(
                requested - self._target_offset_pulses,
                -MAX_TARGET_STEP_PULSES,
                MAX_TARGET_STEP_PULSES,
            )
        )
        if step == 0:
            return False

        self._write(build_position_frame(step, movement_mode=0))
        self._target_offset_pulses += step
        self._last_send_time = now
        self._pending_function = 0xFD
        self._pending_since = now
        return True

    def read_current_position_deg(self, timeout_s=0.080):
        """Read the signed accumulated motor position in degrees.

        Emm_V5.0 replies with:
        address, 0x36, sign, position[31:0], 0x6B.
        This synchronous diagnostic query must not interrupt a pending command.
        """

        if (
            self._serial is None
            or not self._initialized
            or self._pending_function is not None
        ):
            return None

        self._drain_uart()
        self._write(build_current_position_query_frame())
        deadline = _clock() + timeout_s
        while _clock() < deadline:
            self._read_uart()
            while len(self._rx_buffer) >= 2:
                if (
                    self._rx_buffer[0] != MOTOR_ADDRESS
                    or self._rx_buffer[1] != 0x36
                ):
                    del self._rx_buffer[0]
                    continue
                if len(self._rx_buffer) < 8:
                    break
                if self._rx_buffer[7] != CHECKSUM_BYTE:
                    del self._rx_buffer[0]
                    continue

                frame = bytes(self._rx_buffer[:8])
                del self._rx_buffer[:8]
                magnitude = (
                    (frame[3] << 24)
                    | (frame[4] << 16)
                    | (frame[5] << 8)
                    | frame[6]
                )
                position_deg = float(magnitude) * 360.0 / 65536.0
                if frame[2] != 0:
                    position_deg = -position_deg
                self.position_query_count += 1
                return position_deg
            time.sleep(0.001)

        self.error_count += 1
        self._consecutive_errors += 1
        return None

    def emergency_stop(self):
        """Immediately stop and reject further targets until reinitialized."""

        if self._serial is None:
            return False
        stopped = False
        try:
            self._pending_function = None
            self._drain_uart()
            self._write(build_stop_frame())
            stopped = True
        except Exception:
            stopped = False
        self._pending_function = None
        self._initialized = False
        self._healthy = False
        return stopped

    def level_and_stop(self):
        """Return to the boot reference, then stop while keeping the shaft locked."""

        if self._serial is None:
            return
        try:
            self._pending_function = None
            delta = -self._target_offset_pulses
            if delta != 0:
                self._transact(
                    build_position_frame(
                        delta,
                        movement_mode=0,
                        speed_rpm=20,
                        acceleration=POSITION_ACCELERATION,
                    ),
                    0xFD,
                )
                self._target_offset_pulses = 0
                time.sleep(0.200)
            self._transact(build_stop_frame(), 0xFE)
        except Exception:
            try:
                self._write(build_stop_frame())
            except Exception:
                pass

    def best_effort_level(self):
        """Send a non-blocking level command even after RX health is lost."""

        if self._serial is None:
            return
        try:
            delta = -self._target_offset_pulses
            if delta != 0:
                self._write(
                    build_position_frame(
                        delta,
                        movement_mode=0,
                        speed_rpm=20,
                        acceleration=POSITION_ACCELERATION,
                    )
                )
                self._target_offset_pulses = 0
        except Exception:
            try:
                self._write(build_stop_frame())
            except Exception:
                pass

    def close(self):
        if self._owns_serial and self._serial is not None:
            self._serial.close()
        self._serial = None
