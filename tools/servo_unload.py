#!/usr/bin/env python3
"""Run the configured S3 unload sequence through robotd and the ESP32 UART."""

import argparse
import re
import socket
import sys
import time
from pathlib import Path


def read_sequence(config_path):
    text = config_path.read_text(encoding="ascii")
    servo_match = re.search(r"^servo:\s*$([\s\S]*?)(?=^[A-Za-z_]+:|\Z)", text, re.MULTILINE)
    if servo_match is None:
        raise ValueError("missing servo section")

    def read_ints(name):
        match = re.search(r"^\s*{}:\s*\[([^]]*)\]".format(re.escape(name)),
                          servo_match.group(1), re.MULTILINE)
        if match is None:
            raise ValueError("missing servo.{} array".format(name))
        return [int(value.strip()) for value in match.group(1).split(",") if value.strip()]

    pulses = read_ints("unload_pulse_us")
    durations_ms = read_ints("unload_duration_ms")
    if not pulses or len(pulses) != len(durations_ms):
        raise ValueError("unload arrays must be non-empty and have the same length")
    if any(duration < 0 for duration in durations_ms):
        raise ValueError("unload durations must be non-negative")
    return pulses, durations_ms


def request(socket_path, command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(1.0)
        client.connect(socket_path)
        client.sendall((command + "\n").encode("ascii"))
        reply = client.recv(1024).decode(errors="replace").strip()
    if reply.startswith("error:"):
        raise RuntimeError(reply)
    return reply


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/robot.yaml"))
    parser.add_argument("--socket", default="/tmp/robotd.sock")
    parser.add_argument("--period-ms", type=int, default=50,
                        help="ESP32 pulse update period during interpolated segments")
    args = parser.parse_args()
    if args.period_ms <= 0:
        parser.error("--period-ms must be positive")
    try:
        pulses, durations_ms = read_sequence(args.config)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    previous = pulses[-1]
    last_sent = None
    print("S3 unload: {} us -> {}".format(previous, pulses))
    try:
        for target, duration_ms in zip(pulses, durations_ms):
            segment_start = time.monotonic()
            duration_s = duration_ms / 1000.0
            while True:
                elapsed = time.monotonic() - segment_start
                progress = 1.0 if duration_s == 0 else min(1.0, elapsed / duration_s)
                pulse = round(previous + progress * (target - previous))
                if pulse != last_sent:
                    request(args.socket, "s3 pulse {}".format(pulse))
                    last_sent = pulse
                if progress >= 1.0:
                    break
                time.sleep(args.period_ms / 1000.0)
            previous = target
    except (OSError, RuntimeError) as error:
        print("unload aborted: {}".format(error), file=sys.stderr)
        return 1
    try:
        request(args.socket, "s3 release")
    except (OSError, RuntimeError) as error:
        print("unload close complete, but release failed: {}".format(error), file=sys.stderr)
        return 1
    print("S3 unload complete: released after {} us".format(previous))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
