"""UART protocol shared with the MSPM0 H-mission controller."""

HEADER = b"\xAA\x55"
VERSION = 0x02
MAX_PAYLOAD = 20

BALL_STATE = 0x10
MODE_SELECT = 0x30
START = 0x31
STOP = 0x32
MCU_HEARTBEAT = 0x33
MODE_READY = 0x40
STARTED = 0x41
CAM_HEARTBEAT = 0x42
TASK_COMPLETE = 0x43
FAULT = 0x44

BALL_VALID = 1 << 0
BALL_CONFIRMED = 1 << 1


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _u16(value):
    value = int(value) & 0xFFFF
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def _u32(value):
    value = int(value) & 0xFFFFFFFF
    return bytes((value & 0xFF, (value >> 8) & 0xFF,
                  (value >> 16) & 0xFF, (value >> 24) & 0xFF))


def build_packet(message_type, sequence, payload=b""):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((VERSION, message_type, len(payload), sequence & 0xFF)) + payload
    return HEADER + body + bytes((crc8(body),))


def build_event(message_type, sequence, task_id, run_id, result=0, state=0):
    return build_packet(message_type, sequence,
                        bytes((task_id & 0xFF, run_id & 0xFF,
                               result & 0xFF, state & 0xFF)))


def build_ball(sequence, task_id, run_id, flags, position_x10_mm,
               velocity_mm_s, confidence, age_ms, timestamp_ms):
    payload = bytes((task_id & 0xFF, run_id & 0xFF, flags & 0xFF))
    payload += _u16(position_x10_mm)
    payload += _u16(velocity_mm_s)
    payload += bytes((max(0, min(255, int(confidence))),))
    payload += _u16(max(0, min(65535, int(age_ms))))
    payload += _u32(timestamp_ms)
    return build_packet(BALL_STATE, sequence, payload)


def decode_command(packet):
    if packet["version"] != VERSION or len(packet["payload"]) != 10:
        return None
    if packet["type"] not in (MODE_SELECT, START, STOP, MCU_HEARTBEAT):
        return None
    p = packet["payload"]
    target = p[2] | (p[3] << 8)
    if target & 0x8000:
        target -= 0x10000
    timestamp = p[6] | (p[7] << 8) | (p[8] << 16) | (p[9] << 24)
    return {
        "type": packet["type"], "sequence": packet["sequence"],
        "task_id": p[0], "run_id": p[1], "target_x10_mm": target,
        "speed_tier": p[4], "flags": p[5], "timestamp_ms": timestamp,
    }


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0

    def feed(self, data):
        if data:
            self.buffer.extend(data)
        packets = []
        while True:
            while len(self.buffer) >= 2 and self.buffer[:2] != HEADER:
                del self.buffer[0]
            if len(self.buffer) < 7:
                break
            length = self.buffer[4]
            if length > MAX_PAYLOAD:
                self.format_errors += 1
                del self.buffer[0]
                continue
            total = 7 + length
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            del self.buffer[:total]
            if crc8(raw[2:-1]) != raw[-1]:
                self.crc_errors += 1
                continue
            packets.append({
                "version": raw[2], "type": raw[3], "length": raw[4],
                "sequence": raw[5], "payload": raw[6:-1],
            })
        return packets
