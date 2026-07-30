#!/usr/bin/env python3
"""Download the most recent line-follow run log from the MSPM0."""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import serial


FIELDS = [
    "index",
    "timestamp_ms",
    "left_target_rpm",
    "right_target_rpm",
    "left_actual_rpm",
    "right_actual_rpm",
    "left_output",
    "right_output",
    "line_mask",
    "line_error",
    "line_state",
    "yaw_rate_dps",
    "yaw_rate_target_dps",
]


def parse_data(line: str) -> dict[str, int | float] | None:
    if not line.startswith((
        "@RUNLOG,DATA,1,",
        "@RUNLOG,DATA,2,",
        "@RUNLOG,DATA,3,",
    )):
        return None
    parts = line.split(",")
    version = int(parts[2])
    expected_fields = 16 if version >= 3 else (15 if version >= 2 else 14)
    if len(parts) != expected_fields:
        raise ValueError(f"bad RUNLOG field count: {len(parts)}")
    values = [int(value) for value in parts[3:]]
    return {
        "index": values[0],
        "timestamp_ms": values[1],
        "left_target_rpm": values[2] / 10.0,
        "right_target_rpm": values[3] / 10.0,
        "left_actual_rpm": values[4] / 10.0,
        "right_actual_rpm": values[5] / 10.0,
        "left_output": values[6],
        "right_output": values[7],
        "line_mask": values[8],
        "line_error": values[9],
        "line_state": values[10],
        "yaw_rate_dps": values[11] / 10.0 if version >= 2 else 0.0,
        "yaw_rate_target_dps": values[12] / 10.0 if version >= 3 else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="USB-TTL port, e.g. COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output-dir", default="output/run_logs")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / (
        "run_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    )

    rows: list[dict[str, int | float]] = []
    began = False
    deadline = time.monotonic() + args.timeout
    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=1.0) as device:
        device.reset_input_buffer()
        device.write(b"@PIDTEST,DUMP\n")
        device.flush()
        while time.monotonic() < deadline:
            line = device.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("@PIDACK,1,ERROR,"):
                raise RuntimeError(line)
            if line.startswith("@RUNLOG,BEGIN,1,"):
                began = True
                print(line)
                continue
            row = parse_data(line)
            if row is not None:
                rows.append(row)
                continue
            if line.startswith("@RUNLOG,END,1,"):
                print(line)
                break
        else:
            raise TimeoutError("RUNLOG download timed out")

    if not began:
        raise RuntimeError("board did not start a RUNLOG transfer")
    with output_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} samples to {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
