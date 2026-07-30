#!/usr/bin/env python3
"""MSPM0 car speed-loop serial capture and analysis tool."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

try:
    import serial
    from serial import SerialException
    from serial.tools import list_ports
except ImportError as exc:
    print(
        "ERROR: missing dependency pyserial; run "
        r".venv\Scripts\python.exe -m pip install -r requirements.txt",
        file=sys.stderr,
    )
    raise SystemExit(10) from exc


FRAME_PREFIXES = ("@PID,1,", "@PID,2,", "@PID,3,")
STOP_COMMAND = b"@PIDTEST,STOP\n"
PING_COMMAND = b"@PIDTEST,PING\n"
ENCODER_COMMAND = b"@PIDTEST,ENCODER\n"
OUTPUT_LIMIT = 9000.0
MIN_VALID_SAMPLES = 20
HANDSHAKE_TIMEOUT_S = 10.0
CSV_FIELDS = [
    "timestamp_ms",
    "left_target_rpm",
    "right_target_rpm",
    "left_actual_rpm",
    "right_actual_rpm",
    "left_output",
    "right_output",
    "left_error_rpm",
    "right_error_rpm",
    "enabled",
    "line_mask",
    "line_error",
    "line_state",
    "straight_time_ms",
    "curve_time_ms",
    "sharp_time_ms",
    "lost_time_ms",
]


@dataclass(frozen=True)
class Sample:
    timestamp_ms: int
    left_target_rpm: float
    right_target_rpm: float
    left_actual_rpm: float
    right_actual_rpm: float
    left_output: int
    right_output: int
    left_error_rpm: float
    right_error_rpm: float
    enabled: int
    line_mask: int = 0
    line_error: int = 0
    line_state: int = 0
    straight_time_ms: int = 0
    curve_time_ms: int = 0
    sharp_time_ms: int = 0
    lost_time_ms: int = 0


def parse_frame(text: str) -> Sample | None:
    if not text.startswith(FRAME_PREFIXES):
        return None
    parts = text.strip().split(",")
    version = int(parts[1])
    expected_fields = {1: 12, 2: 18, 3: 19}.get(version)
    if expected_fields is None:
        raise ValueError(f"unsupported protocol version {version}")
    if len(parts) != expected_fields:
        raise ValueError(
            f"expected {expected_fields} fields for protocol {version}, "
            f"got {len(parts)}"
        )
    values = parts[2:]
    sample = Sample(
        timestamp_ms=int(values[0]),
        left_target_rpm=float(values[1]),
        right_target_rpm=float(values[2]),
        left_actual_rpm=float(values[3]),
        right_actual_rpm=float(values[4]),
        left_output=int(values[5]),
        right_output=int(values[6]),
        left_error_rpm=float(values[7]),
        right_error_rpm=float(values[8]),
        enabled=int(values[9]),
        line_mask=int(values[10]) if version >= 2 else 0,
        line_error=int(values[11]) if version >= 2 else 0,
        line_state=int(values[12]) if version >= 2 else 0,
        straight_time_ms=int(values[13]) if version >= 2 else 0,
        curve_time_ms=int(values[14]) if version >= 3 else 0,
        sharp_time_ms=int(values[15] if version >= 3 else values[14])
        if version >= 2 else 0,
        lost_time_ms=int(values[16] if version >= 3 else values[15])
        if version >= 2 else 0,
    )
    numeric = asdict(sample).values()
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError("non-finite numeric value")
    if sample.enabled not in (0, 1):
        raise ValueError("enabled must be 0 or 1")
    if not 0 <= sample.line_mask <= 0xFF:
        raise ValueError("line_mask must be 0..255")
    valid_states = (0, 1, 2, 3) if version >= 3 else (0, 1, 2)
    if sample.line_state not in valid_states:
        raise ValueError("invalid line_state for protocol version")
    return sample


def scan_ports() -> list:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        print("No serial ports found.")
        return ports
    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        identity = " | ".join(
            value for value in (port.description, port.manufacturer, port.hwid)
            if value and value != "n/a"
        )
        print(f"  [{index}] {port.device}: {identity}")
    return ports


def choose_port(requested: str | None, non_interactive: bool) -> str:
    ports = scan_ports()
    if requested:
        return requested
    if not ports:
        raise RuntimeError("no serial port found")
    if len(ports) == 1:
        print(f"Using the only detected port: {ports[0].device}")
        return ports[0].device
    if non_interactive or not sys.stdin.isatty():
        names = ", ".join(port.device for port in ports)
        raise RuntimeError(f"multiple ports found ({names}); specify --port")
    while True:
        answer = input("Select port number: ").strip()
        if answer.isdigit() and 1 <= int(answer) <= len(ports):
            return ports[int(answer) - 1].device
        print("Invalid selection.")


def _mean_abs(values: Iterable[float]) -> float:
    data = list(values)
    return statistics.fmean(abs(value) for value in data) if data else 0.0


def _wheel_metrics(
    samples: Sequence[Sample], side: str, target: float
) -> dict[str, float | None]:
    actual = [getattr(sample, f"{side}_actual_rpm") for sample in samples]
    error = [getattr(sample, f"{side}_error_rpm") for sample in samples]
    tail_count = max(5, len(samples) // 5)
    tail_actual = actual[-tail_count:]
    tail_error = error[-tail_count:]
    target_abs = abs(target)
    direction = 1.0 if target >= 0 else -1.0
    directed_peak = max((value * direction for value in actual), default=0.0)
    overshoot = (
        max(0.0, directed_peak - target_abs) / target_abs * 100.0
        if target_abs > 0.01 else 0.0
    )
    tolerance = max(0.05 * target_abs, 1.0)
    settling_ms: float | None = None
    for index in range(len(samples)):
        if all(abs(value) <= tolerance for value in error[index:]):
            settling_ms = float(
                samples[index].timestamp_ms - samples[0].timestamp_ms
            )
            break
    return {
        "steady_state_error_rpm": statistics.fmean(tail_error),
        "steady_state_abs_error_rpm": _mean_abs(tail_error),
        "overshoot_percent": overshoot,
        "settling_time_ms": settling_ms,
        "speed_stddev_rpm": (
            statistics.pstdev(tail_actual) if len(tail_actual) > 1 else 0.0
        ),
    }


def analyze(samples: Sequence[Sample]) -> dict:
    active = [sample for sample in samples if sample.enabled]
    if len(active) < MIN_VALID_SAMPLES:
        raise RuntimeError(
            f"insufficient valid samples: {len(active)} < {MIN_VALID_SAMPLES}"
        )
    timestamps = [sample.timestamp_ms for sample in active]
    deltas = [
        current - previous
        for previous, current in zip(timestamps, timestamps[1:])
        if current > previous
    ]
    elapsed_s = (timestamps[-1] - timestamps[0]) / 1000.0
    sample_rate = (
        (len(active) - 1) / elapsed_s if elapsed_s > 0.0 else 0.0
    )
    expected_period = statistics.median(deltas) if deltas else 0.0
    valid_intervals = sum(
        1 for delta in deltas
        if expected_period > 0 and delta <= expected_period * 1.5
    )
    effective_rate = (
        sample_rate * valid_intervals / len(deltas) if deltas else 0.0
    )
    left_target = statistics.median(
        sample.left_target_rpm for sample in active
    )
    right_target = statistics.median(
        sample.right_target_rpm for sample in active
    )
    left = _wheel_metrics(active, "left", left_target)
    right = _wheel_metrics(active, "right", right_target)
    wheel_difference = statistics.fmean(
        abs(sample.left_actual_rpm - sample.right_actual_rpm)
        for sample in active
    )
    saturated = sum(
        1 for sample in active
        if abs(sample.left_output) >= OUTPUT_LIMIT * 0.98
        or abs(sample.right_output) >= OUTPUT_LIMIT * 0.98
    )
    saturation_percent = saturated / len(active) * 100.0
    target_scale = max(abs(left_target), abs(right_target), 1.0)
    error_ratio = (
        float(left["steady_state_abs_error_rpm"])
        + float(right["steady_state_abs_error_rpm"])
    ) / (2.0 * target_scale)
    overshoot_ratio = (
        float(left["overshoot_percent"]) + float(right["overshoot_percent"])
    ) / 200.0
    fluctuation_ratio = (
        float(left["speed_stddev_rpm"]) + float(right["speed_stddev_rpm"])
    ) / (2.0 * target_scale)
    mismatch_ratio = wheel_difference / target_scale
    score = 100.0
    score -= min(35.0, error_ratio * 100.0)
    score -= min(20.0, overshoot_ratio * 40.0)
    score -= min(15.0, fluctuation_ratio * 60.0)
    score -= min(15.0, mismatch_ratio * 40.0)
    score -= min(15.0, saturation_percent * 0.5)
    return {
        "status": "ok",
        "valid_samples": len(active),
        "duration_s": elapsed_s,
        "target_rpm": {"left": left_target, "right": right_target},
        "left": left,
        "right": right,
        "mean_wheel_speed_difference_rpm": wheel_difference,
        "output_saturation_percent": saturation_percent,
        "nominal_sample_rate_hz": sample_rate,
        "effective_sample_rate_hz": effective_rate,
        "median_sample_period_ms": expected_period,
        "line_state_time_ms": {
            "straight": active[-1].straight_time_ms,
            "curve": active[-1].curve_time_ms,
            "sharp": active[-1].sharp_time_ms,
            "lost": active[-1].lost_time_ms,
        },
        "composite_score": round(max(0.0, score), 1),
        "score_note": "diagnostic 0-100 score; compare runs under identical conditions",
    }


def save_csv(path: Path, samples: Sequence[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)


def print_summary(result: dict) -> None:
    left, right = result["left"], result["right"]
    print("\nPID speed-loop analysis")
    print(
        f"  samples/rate: {result['valid_samples']} / "
        f"{result['effective_sample_rate_hz']:.2f} Hz effective"
    )
    print(
        "  steady-state abs error L/R: "
        f"{left['steady_state_abs_error_rpm']:.3f} / "
        f"{right['steady_state_abs_error_rpm']:.3f} rpm"
    )
    print(
        f"  overshoot L/R: {left['overshoot_percent']:.2f}% / "
        f"{right['overshoot_percent']:.2f}%"
    )
    print(
        f"  settling L/R: {left['settling_time_ms']} / "
        f"{right['settling_time_ms']} ms"
    )
    print(
        f"  speed stddev L/R: {left['speed_stddev_rpm']:.3f} / "
        f"{right['speed_stddev_rpm']:.3f} rpm"
    )
    print(
        "  mean wheel difference / output saturation: "
        f"{result['mean_wheel_speed_difference_rpm']:.3f} rpm / "
        f"{result['output_saturation_percent']:.2f}%"
    )
    states = result["line_state_time_ms"]
    print(
        "  line state ms straight/curve/sharp/lost: "
        f"{states['straight']} / {states['curve']} / "
        f"{states['sharp']} / {states['lost']}"
    )
    print(f"  composite score: {result['composite_score']}/100")


def run_capture(args: argparse.Namespace) -> int:
    port = choose_port(args.port, args.non_interactive)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("pid_%Y%m%d_%H%M%S")
    raw_path = output_dir / f"{stem}.log"
    csv_path = output_dir / f"{stem}.csv"
    result_path = output_dir / f"{stem}.json"
    samples: list[Sample] = []
    format_errors = 0
    display_sample_counter = 0
    last_data_at = time.monotonic()
    started_at = time.monotonic()
    max_runtime = args.duration + args.start_delay_allowance + 5.0
    device = None

    try:
        device = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0.2,
            write_timeout=1.0,
        )
        device.reset_input_buffer()
        device.write(PING_COMMAND)
        device.flush()
        handshake_deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        next_ping_at = time.monotonic() + 1.0
        handshake_ready = False
        while time.monotonic() < handshake_deadline:
            reply = device.readline()
            if not reply:
                if time.monotonic() >= next_ping_at:
                    device.write(PING_COMMAND)
                    device.flush()
                    next_ping_at = time.monotonic() + 1.0
                continue
            reply_text = reply.decode("utf-8", errors="replace").strip()
            print(reply_text)
            if reply_text.startswith("@PIDACK,1,READY,"):
                handshake_ready = True
                break
        if not handshake_ready:
            raise RuntimeError(
                "board did not acknowledge PID test protocol; "
                "START was not sent (check firmware, UART wiring, and baud rate)"
            )
        if args.kp is not None:
            gains_command = (
                f"@PIDTEST,GAINS,{args.kp:.3f},{args.ki:.3f},{args.kd:.3f}\n"
            )
            device.write(gains_command.encode("ascii"))
            device.flush()
            gains_deadline = time.monotonic() + 2.0
            gains_ack = False
            while time.monotonic() < gains_deadline:
                reply = device.readline()
                if not reply:
                    continue
                reply_text = reply.decode("utf-8", errors="replace").strip()
                print(reply_text)
                if reply_text.startswith("@PIDACK,1,GAINS,"):
                    gains_ack = True
                    break
                if reply_text.startswith("@PIDACK,1,ERROR,"):
                    raise RuntimeError(f"board rejected PID gains: {reply_text}")
            if not gains_ack:
                raise RuntimeError("board did not confirm PID gains; START was not sent")
        command = (
            f"@PIDTEST,START,{args.target:.3f},"
            f"{min(120000, int((args.duration + args.start_delay_allowance) * 1000))}\n"
        )
        device.write(command.encode("ascii"))
        device.flush()
        print(f"Capturing {port} at {args.baud} baud -> {output_dir}")
        with raw_path.open("w", encoding="utf-8", errors="replace") as raw:
            while time.monotonic() - started_at < max_runtime:
                if time.monotonic() - started_at >= args.duration:
                    break
                data = device.readline()
                if not data:
                    if time.monotonic() - last_data_at > args.no_data_timeout:
                        raise RuntimeError(
                            f"no serial data for {args.no_data_timeout:.1f}s"
                        )
                    continue
                text = data.decode("utf-8", errors="replace").strip()
                raw.write(text + "\n")
                raw.flush()
                if text.startswith(FRAME_PREFIXES):
                    try:
                        sample = parse_frame(text)
                    except (ValueError, OverflowError):
                        format_errors += 1
                        continue
                    if sample is not None:
                        samples.append(sample)
                        last_data_at = time.monotonic()
                        display_sample_counter += 1
                        if display_sample_counter % 10 == 0:
                            print(text)
                elif text.startswith("@PIDACK,"):
                    print(text)
                    last_data_at = time.monotonic()
                elif text:
                    print(text)
    except SerialException as exc:
        message = str(exc)
        hint = (
            " Close VS Code serial monitor or other terminal and retry."
            if "access" in message.lower() or "denied" in message.lower()
            else " Check USB connection and board power."
        )
        raise RuntimeError(f"serial open/read failed: {message}.{hint}") from exc
    finally:
        if device is not None and device.is_open:
            for _ in range(3):
                try:
                    device.write(STOP_COMMAND)
                    device.flush()
                except SerialException:
                    break
                time.sleep(0.05)
            device.close()

    save_csv(csv_path, samples)
    result = analyze(samples)
    result.update(
        {
            "port": port,
            "baud": args.baud,
            "raw_log": str(raw_path.resolve()),
            "csv": str(csv_path.resolve()),
            "format_errors": format_errors,
            "pid_gains": (
                {"kp": args.kp, "ki": args.ki, "kd": args.kd}
                if args.kp is not None else None
            ),
        }
    )
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print_summary(result)
    print("RESULT_JSON=" + json.dumps(result, ensure_ascii=False))
    print(f"Result file: {result_path.resolve()}")
    return 0


def set_board_gains(port: str, baud: int, gains: tuple[float, float, float]) -> None:
    kp, ki, kd = gains
    with serial.Serial(port, baud, timeout=0.2, write_timeout=1.0) as device:
        device.reset_input_buffer()
        device.write(PING_COMMAND)
        device.flush()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        next_ping_at = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            text = device.readline().decode("utf-8", errors="replace").strip()
            if text.startswith("@PIDACK,1,READY,"):
                break
            if time.monotonic() >= next_ping_at:
                device.write(PING_COMMAND)
                device.flush()
                next_ping_at = time.monotonic() + 1.0
        else:
            raise RuntimeError("board did not acknowledge before applying best gains")
        device.write(
            f"@PIDTEST,GAINS,{kp:.3f},{ki:.3f},{kd:.3f}\n".encode("ascii")
        )
        device.flush()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            text = device.readline().decode("utf-8", errors="replace").strip()
            if text.startswith("@PIDACK,1,GAINS,"):
                device.write(STOP_COMMAND)
                device.flush()
                return
            if text.startswith("@PIDACK,1,ERROR,"):
                raise RuntimeError(f"board rejected best gains: {text}")
    raise RuntimeError("board did not confirm best gains")


def run_auto_tune(args: argparse.Namespace) -> int:
    port = choose_port(args.port, True)
    base_kp = args.kp if args.kp is not None else 10.0
    base_ki = args.ki if args.ki is not None else 1.0
    base_kd = args.kd if args.kd is not None else 0.0
    candidates = [
        (base_kp, base_ki, base_kd),
        (min(30.0, base_kp + 2.0), base_ki, base_kd),
        (min(30.0, base_kp + 4.0), base_ki, base_kd),
        (min(30.0, base_kp + 2.0), max(0.0, base_ki - 0.2), base_kd),
        (min(30.0, base_kp + 4.0), max(0.0, base_ki - 0.2), base_kd),
        (min(30.0, base_kp + 2.0), base_ki, min(2.0, base_kd + 0.15)),
    ][: args.max_runs]
    session_dir = (
        Path(args.output_dir)
        / ("autotune_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    )
    session_dir.mkdir(parents=True, exist_ok=True)
    results = []
    print(
        f"Auto-tune: {len(candidates)} bounded runs, target={args.target:g} rpm, "
        f"{args.duration:g}s each. Keep wheels raised and stay at the power switch."
    )
    for index, (kp, ki, kd) in enumerate(candidates, start=1):
        run_dir = session_dir / f"run_{index:02d}_kp{kp:g}_ki{ki:g}_kd{kd:g}"
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--port", port,
            "--baud", str(args.baud),
            "--duration", str(args.duration),
            "--target", str(args.target),
            "--output-dir", str(run_dir),
            "--non-interactive",
            "--kp", str(kp),
            "--ki", str(ki),
            "--kd", str(kd),
        ]
        print(f"\n=== AUTO RUN {index}/{len(candidates)}: Kp={kp}, Ki={ki}, Kd={kd} ===")
        completed = subprocess.run(command, check=False)
        if completed.returncode != 0:
            print("Auto-tune aborted after a failed run; stop was attempted.")
            return 2
        json_files = sorted(run_dir.glob("*.json"))
        if not json_files:
            print("Auto-tune aborted: run produced no JSON result.", file=sys.stderr)
            return 2
        result = json.loads(json_files[-1].read_text(encoding="utf-8"))
        results.append(
            {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "score": result["composite_score"],
                "result_file": str(json_files[-1].resolve()),
            }
        )
    best = max(results, key=lambda item: item["score"])
    set_board_gains(port, args.baud, (best["kp"], best["ki"], best["kd"]))
    summary = {
        "status": "ok",
        "mode": "auto_tune",
        "target_rpm": args.target,
        "runs": results,
        "best": best,
        "note": "best gains applied to RAM; reset restores firmware defaults",
    }
    summary_path = session_dir / "autotune_result.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        f"\nBEST: Kp={best['kp']}, Ki={best['ki']}, Kd={best['kd']}, "
        f"score={best['score']}. Applied to board RAM."
    )
    print("RESULT_JSON=" + json.dumps(summary, ensure_ascii=False))
    return 0


def run_encoder_test(args: argparse.Namespace) -> int:
    port = choose_port(args.port, args.non_interactive)
    device = None
    rows: list[list[int]] = []
    try:
        device = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0.2,
            write_timeout=1.0,
        )
        device.reset_input_buffer()
        device.write(PING_COMMAND)
        device.flush()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        next_ping_at = time.monotonic() + 1.0
        ready = False
        while time.monotonic() < deadline:
            text = device.readline().decode("utf-8", errors="replace").strip()
            if text.startswith("@PIDACK,1,READY,"):
                ready = True
                break
            if time.monotonic() >= next_ping_at:
                device.write(PING_COMMAND)
                device.flush()
                next_ping_at = time.monotonic() + 1.0
        if not ready:
            raise RuntimeError("board did not acknowledge PID test protocol")
        device.write(ENCODER_COMMAND)
        device.flush()
        print(
            f"Encoder test on {port}: motor output is disabled. "
            f"Turn both wheels by hand for {args.duration:g}s."
        )
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            data = device.readline()
            if not data:
                continue
            text = data.decode("utf-8", errors="replace").strip()
            if text.startswith("@PIDACK,"):
                print(text)
                continue
            if not text.startswith("@ENC,1,"):
                continue
            parts = text.split(",")
            if len(parts) != 11:
                continue
            try:
                row = [int(value) for value in parts[2:]]
            except ValueError:
                continue
            rows.append(row)
            print(
                f"M1 count={row[1]:7d} A={row[2]:7d} B={row[3]:7d} "
                f"state={row[4]:02b} | M2 count={row[5]:7d} "
                f"A={row[6]:7d} B={row[7]:7d} state={row[8]:02b}"
            )
    except SerialException as exc:
        raise RuntimeError(f"serial encoder test failed: {exc}") from exc
    finally:
        if device is not None and device.is_open:
            try:
                device.write(STOP_COMMAND)
                device.flush()
            except SerialException:
                pass
            device.close()
    if len(rows) < 2:
        raise RuntimeError("no valid encoder diagnostic frames received")
    first, last = rows[0], rows[-1]
    result = {
        "status": "ok",
        "mode": "encoder_test",
        "motor1": {
            "count_delta": last[1] - first[1],
            "a_edge_delta": last[2] - first[2],
            "b_edge_delta": last[3] - first[3],
        },
        "motor2": {
            "count_delta": last[5] - first[5],
            "a_edge_delta": last[6] - first[6],
            "b_edge_delta": last[7] - first[7],
        },
    }
    print("RESULT_JSON=" + json.dumps(result, ensure_ascii=False))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture and analyze the car speed-loop PID telemetry."
    )
    parser.add_argument("--list", action="store_true", help="list serial ports")
    parser.add_argument("--port", help="serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--target", type=float, default=25.0, help="target rpm")
    parser.add_argument("--output-dir", default=str(Path(__file__).parent / "logs"))
    parser.add_argument("--no-data-timeout", type=float, default=3.0)
    parser.add_argument("--start-delay-allowance", type=float, default=2.0)
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument(
        "--self-test", action="store_true", help="test parser and analyzer"
    )
    parser.add_argument(
        "--encoder-test",
        action="store_true",
        help="disable motors and report raw A/B encoder edge counts",
    )
    parser.add_argument("--kp", type=float, help="runtime proportional gain")
    parser.add_argument("--ki", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument(
        "--auto-tune",
        action="store_true",
        help="run a bounded PID candidate search and apply the best gains to RAM",
    )
    parser.add_argument("--max-runs", type=int, default=6)
    return parser


def self_test() -> int:
    samples = []
    for index in range(100):
        target = 25.0
        actual = target * (1.0 - math.exp(-index / 12.0))
        frame = (
            f"@PID,1,{index * 20},{target},{target},{actual},{actual},"
            f"2500,2510,{target - actual},{target - actual},1"
        )
        sample = parse_frame(frame)
        assert sample is not None
        samples.append(sample)
    result = analyze(samples)
    assert result["valid_samples"] == 100
    assert parse_frame("boot message") is None
    print("SELF_TEST_OK")
    print("RESULT_JSON=" + json.dumps(result))
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 120:
        parser.error("--duration must be in (0, 120]")
    if abs(args.target) > 100:
        parser.error("--target must be in [-100, 100]")
    if args.no_data_timeout <= 0:
        parser.error("--no-data-timeout must be positive")
    if args.kp is not None and not (0 <= args.kp <= 30):
        parser.error("--kp must be in [0, 30]")
    if not (0 <= args.ki <= 3):
        parser.error("--ki must be in [0, 3]")
    if not (0 <= args.kd <= 2):
        parser.error("--kd must be in [0, 2]")
    if not (1 <= args.max_runs <= 6):
        parser.error("--max-runs must be in [1, 6]")
    if args.self_test:
        return self_test()
    if args.encoder_test:
        try:
            return run_encoder_test(args)
        except (RuntimeError, OSError) as exc:
            failure = {"status": "error", "message": str(exc)}
            print(f"ERROR: {exc}", file=sys.stderr)
            print("RESULT_JSON=" + json.dumps(failure, ensure_ascii=False))
            return 2
    if args.auto_tune:
        try:
            return run_auto_tune(args)
        except (RuntimeError, OSError, SerialException) as exc:
            failure = {"status": "error", "message": str(exc)}
            print(f"ERROR: {exc}", file=sys.stderr)
            print("RESULT_JSON=" + json.dumps(failure, ensure_ascii=False))
            return 2
    if args.list:
        scan_ports()
        return 0
    try:
        return run_capture(args)
    except (RuntimeError, OSError) as exc:
        failure = {"status": "error", "message": str(exc)}
        print(f"ERROR: {exc}", file=sys.stderr)
        print("RESULT_JSON=" + json.dumps(failure, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted; stop command was attempted.", file=sys.stderr)
        raise SystemExit(130)
