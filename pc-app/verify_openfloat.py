"""Check an OpenFloat serial capture or a live COM port without a GUI."""

import argparse
from contextlib import ExitStack
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

from openfloat_protocol import DeviceStats, LiveSample, ShotEvent, decode_text, missing_notifications, text_lines


def analyze_lines(lines):
    """Consume lines incrementally; memory use is independent of capture length."""
    result = {
        "live_frames": 0, "shot_events": 0, "malformed_frames": 0,
        "ignored_lines": 0, "missing_notifications": 0,
        "sequence_discontinuities": 0, "nonunit_quaternions": 0,
        "dt_us_min": None, "dt_us_max": None, "dt_us_mean": None,
        "received_rate_hz": None,
        "device_stats_frames": 0, "latest_device_counters": None,
    }
    previous = None
    dt_sum = interval_count = interval_us = 0
    for line in lines:
        try:
            sample = decode_text(line)
        except ValueError:
            result["malformed_frames"] += 1
            continue
        if isinstance(sample, DeviceStats):
            result["device_stats_frames"] += 1
            result["latest_device_counters"] = asdict(sample)
            continue
        if isinstance(sample, ShotEvent):
            result["shot_events"] += 1
            continue
        if not isinstance(sample, LiveSample):
            result["ignored_lines"] += 1
            continue
        result["live_frames"] += 1
        dt_sum += sample.dt_us
        result["dt_us_min"] = sample.dt_us if result["dt_us_min"] is None else min(result["dt_us_min"], sample.dt_us)
        result["dt_us_max"] = sample.dt_us if result["dt_us_max"] is None else max(result["dt_us_max"], sample.dt_us)
        norm = math.sqrt(sum(v * v for v in sample.quaternion_1e6)) / 1_000_000
        if abs(norm - 1) > 0.01:
            result["nonunit_quaternions"] += 1
        if previous is not None:
            missing = missing_notifications(previous.sequence, sample.sequence, bits=32)
            delta_us = sample.uptime_us - previous.uptime_us
            if missing is None or delta_us <= 0:
                result["sequence_discontinuities"] += 1
            else:
                result["missing_notifications"] += missing
                interval_count += 1
                interval_us += delta_us
        previous = sample
    if result["live_frames"]:
        result["dt_us_mean"] = dt_sum / result["live_frames"]
    if interval_us:
        result["received_rate_hz"] = interval_count * 1_000_000 / interval_us
    return result


def positive_seconds(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("duration must be finite and greater than zero")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="Raw serial text capture to inspect.")
    source.add_argument("--port", help="COM port to capture with DTR/RTS asserted.")
    parser.add_argument("--seconds", type=positive_seconds, default=10.0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--min-frames", type=int, default=10)
    parser.add_argument("--capture", type=Path, help="Save raw live serial output to a NEW file.")
    args = parser.parse_args(argv)
    if args.min_frames < 1 or args.baud < 1:
        parser.error("--min-frames and --baud must be positive")
    if args.capture and not args.port:
        parser.error("--capture requires --port")
    try:
        with ExitStack() as stack:
            if args.input:
                capture_input = stack.enter_context(args.input.open("rb"))
                lines = text_lines(iter(lambda: capture_input.read(8192), b""))
            else:
                try:
                    import serial
                except ImportError:
                    parser.error("live capture requires pyserial; the installed NCS Python includes it")
                capture = stack.enter_context(args.capture.open("xb")) if args.capture else None
                port = serial.Serial(port=None, baudrate=args.baud, timeout=0.2)
                stack.callback(port.close)
                port.port = args.port
                port.dtr = port.rts = True
                port.open()
                deadline = time.monotonic() + args.seconds

                def serial_chunks():
                    while time.monotonic() < deadline:
                        raw = port.read_until(b"\n", size=513)
                        if raw:
                            if capture:
                                capture.write(raw)
                            yield raw

                lines = text_lines(serial_chunks())
            report = analyze_lines(lines)
    except (OSError, ValueError) as error:
        print(f"Capture failed: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    print(json.dumps(report, indent=2))
    return int(report["live_frames"] < args.min_frames or report["malformed_frames"] > 0
               or report["nonunit_quaternions"] > 0)


if __name__ == "__main__":
    raise SystemExit(main())
