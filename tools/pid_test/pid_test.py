#!/usr/bin/env python3
"""MSPM0 car speed-loop serial capture and analysis tool."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
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


FRAME_PREFIX = "@PID,1,"
STOP_COMMAND = b"@PIDTEST,STOP\n"
OUTPUT_LIMIT = 8000.0
MIN_VALID_SAMPLES = 20
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


def parse_frame(text: str) -> Sample | None:
    if not text.startswith(FRAME_PREFIX):
        return None
    parts = text.strip().split(",")
    if len(parts) != 12:
        raise ValueError(f"expected 12 fields, got {len(parts)}")
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
    )
    numeric = asdict(sample).values()
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError("non-finite numeric value")
    if sample.enabled not in (0, 1):
        raise ValueError("enabled must be 0 or 1")
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
                print(text)
                if text.startswith(FRAME_PREFIX):
                    try:
                        sample = parse_frame(text)
                    except (ValueError, OverflowError):
                        format_errors += 1
                        continue
                    if sample is not None:
                        samples.append(sample)
                        last_data_at = time.monotonic()
                elif text.startswith("@PIDACK,"):
                    last_data_at = time.monotonic()
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
    if args.self_test:
        return self_test()
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
