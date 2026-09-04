#!/usr/bin/env python3
"""Supervise robotbrain with fresh calibrated YOLO frames.

This process is deliberately separate from the dashboard.  It sends frames to
robotbrain only while the bridge keeps atomically updating a protocol file.
On a stale frame, malformed frame, or termination it stops robotbrain, whose
live-mode exit path commands ``ga25 0`` and ``stop`` through robotd.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import socket


def write_status(path: Path, **fields) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".robot-mission-", dir=path.parent, text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(fields, output, ensure_ascii=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def valid_frame(line: str) -> bool:
    fields = line.split()
    if len(fields) < 2 or fields[0] not in {"0", "1"} or fields[1] not in {"0", "1"}:
        return False
    index = 2
    while index < len(fields):
        if index + 4 > len(fields):
            return False
        # CLASS CONFIDENCE CENTER_X BOTTOM_Y
        index += 4
        if index < len(fields) and fields[index] == "@":
            if index + 3 > len(fields):
                return False
            try:
                float(fields[index + 1])
                float(fields[index + 2])
            except ValueError:
                return False
            index += 3
    return True


def terminate(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


def robotd_request(socket_path: str, command: str) -> None:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(.3)
        client.connect(socket_path)
        client.sendall((command + "\n").encode())
        reply = client.recv(1024).decode(errors="replace")
    if reply.startswith("error:"):
        raise RuntimeError(reply)


def ramp_toward(current: float, target: float, step: float) -> float:
    """Smooth increases, while applying braking and sign reversals promptly."""
    if current * target < 0:
        return current - math.copysign(min(abs(current), step), current)
    if abs(target) <= abs(current):
        return target
    return current + max(-step, min(step, target - current))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robotbrain", required=True)
    parser.add_argument("--protocol-file", default="/tmp/robotvision-frame.txt")
    parser.add_argument("--status-file", default="/tmp/robot-mission-status.json")
    parser.add_argument("--expected-objects", type=int, default=2)
    parser.add_argument("--socket", default="/tmp/robotd.sock")
    parser.add_argument("--max-frame-age", type=float, default=1.50)
    parser.add_argument("--heartbeat-seconds", type=float, default=.08)
    parser.add_argument("--linear-rise-step", type=float, default=.030)
    parser.add_argument("--yaw-rise-step", type=float, default=.120)
    parser.add_argument("--poll-seconds", type=float, default=.04)
    args = parser.parse_args()
    if min(args.expected_objects, args.max_frame_age, args.poll_seconds, args.heartbeat_seconds,
           args.linear_rise_step, args.yaw_rise_step) <= 0:
        raise SystemExit("mission timing and smoothing values must be positive")

    protocol_file = Path(args.protocol_file)
    status_file = Path(args.status_file)
    command = [args.robotbrain, "--live", "--expected-objects", str(args.expected_objects)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
    state = "starting"
    last_output = ""
    last_command = None
    command_lock = threading.Lock()

    def read_output() -> None:
        nonlocal state, last_output, last_command
        assert process.stdout is not None
        for line in process.stdout:
            last_output = line.strip()
            if last_output.startswith("state="):
                state = last_output.split()[0].removeprefix("state=")
                fields = dict(token.split("=", 1) for token in last_output.split() if "=" in token)
                twist = fields.get("twist", "").split(",")
                if len(twist) == 3 and "ga25" in fields:
                    try:
                        command = (float(twist[0]), float(twist[1]), float(twist[2]), int(fields["ga25"]))
                    except ValueError:
                        continue
                    with command_lock:
                        last_command = command

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    last_timestamp_ns: int | None = None
    last_heartbeat = 0.0
    applied_forward = applied_left = applied_yaw = 0.0
    error = ""
    try:
        while process.poll() is None:
            try:
                stat = protocol_file.stat()
                frame_age = time.time() - stat.st_mtime
            except FileNotFoundError:
                raise RuntimeError(f"waiting for vision frame: {protocol_file}")
            if frame_age > args.max_frame_age:
                raise RuntimeError(f"vision frame stale for {frame_age:.2f}s")
            if stat.st_mtime_ns != last_timestamp_ns:
                line = protocol_file.read_text(encoding="utf-8").strip()
                if not valid_frame(line):
                    raise RuntimeError("invalid vision protocol frame")
                assert process.stdin is not None
                process.stdin.write(line + "\n")
                process.stdin.flush()
                last_timestamp_ns = stat.st_mtime_ns
            now = time.monotonic()
            if now - last_heartbeat >= args.heartbeat_seconds:
                with command_lock:
                    command = last_command
                if command is not None:
                    forward, left, yaw, collector = command
                    applied_forward = ramp_toward(applied_forward, forward, args.linear_rise_step)
                    applied_left = ramp_toward(applied_left, left, args.linear_rise_step)
                    applied_yaw = ramp_toward(applied_yaw, yaw, args.yaw_rise_step)
                    robotd_request(args.socket, f"twist {applied_forward:.3f} {applied_left:.3f} {applied_yaw:.3f}")
                    robotd_request(args.socket, f"ga25 {collector}")
                last_heartbeat = now
            write_status(status_file, running=True, state=state, frame_age_ms=round(frame_age * 1000),
                         last_output=last_output, error="")
            time.sleep(args.poll_seconds)
        if process.returncode:
            error = last_output or f"robotbrain exited with {process.returncode}"
    except (OSError, RuntimeError, ValueError) as exception:
        error = str(exception)
    finally:
        terminate(process)
        reader.join(timeout=.2)
        write_status(status_file, running=False, state=state, frame_age_ms=None,
                     last_output=last_output, error=error)
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
