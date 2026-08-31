#!/usr/bin/env python3
"""Keyboard teleoperation through the local robotd Unix socket."""

import argparse
import re
import select
import socket
import sys
import termios
import time
import tty


def request(socket_path, command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(.5)
        client.connect(socket_path)
        client.sendall((command + "\n").encode())
        return client.recv(1024).decode(errors="replace").strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/robotd.sock")
    parser.add_argument("--linear", type=float, default=.10,
                        help="linear velocity increment per key press in m/s")
    parser.add_argument("--yaw", type=float, default=.50,
                        help="yaw-rate increment per key press in rad/s")
    parser.add_argument("--max-linear", type=float, default=.40,
                        help="maximum absolute linear velocity in m/s")
    parser.add_argument("--max-yaw", type=float, default=2.00,
                        help="maximum absolute yaw rate in rad/s")
    parser.add_argument("--period", type=float, default=.08,
                        help="twist refresh period in seconds")
    parser.add_argument("--servo-pulse-step", type=int, default=25,
                        help="S3 raw-pulse increment in microseconds")
    args = parser.parse_args()
    if (args.linear <= 0 or args.yaw <= 0 or args.max_linear <= 0 or
            args.max_yaw <= 0 or args.period <= 0 or args.servo_pulse_step <= 0):
        parser.error("all control increments and period must be positive")

    print("W/S forward/back, A/D left/right, Q/E yaw: each press adds speed")
    print(f"limits: linear +-{args.max_linear:.2f} m/s, yaw +-{args.max_yaw:.2f} rad/s")
    print("Space stop, Esc quit")
    print("[ / ] S3 pulse (800..2125 us), R releases S3")
    print("F/B GA25 forward/reverse, ,/. GA25 speed down/up, G stops GA25, T state")
    vx = vy = wz = 0.0
    servo_pulse_us = 1500
    ga25_speed = 0
    ga25_direction = 1
    old_settings = termios.tcgetattr(sys.stdin)

    def issue(command, show=True):
        try:
            reply = request(args.socket, command)
            if show:
                print(reply)
            return reply
        except OSError as error:
            print(f"robotd error: {error}", file=sys.stderr)
            return None

    def show_s3_status():
        nonlocal servo_pulse_us
        reply = issue("s3", show=False)
        if reply is None:
            return
        mode_match = re.search(r"^s3 (\w+)", reply)
        raw_pulse_match = re.search(r"raw (\d+)us", reply)
        current_match = re.search(r"current (\d+)deg", reply)
        target_match = re.search(r"target (\d+)deg", reply)
        current_pulse_match = re.search(r"current \d+deg (\d+)us", reply)
        duty_match = re.search(r"(\d+\.\d+)%", reply)
        moving_match = re.search(r"moving (\d+)", reply)
        pulse_match = raw_pulse_match or current_pulse_match
        if pulse_match is None:
            print(reply)
            return
        servo_pulse_us = int(pulse_match.group(1))
        mode = mode_match.group(1) if mode_match else "unknown"
        duty = f", {duty_match.group(1)}%" if duty_match else ""
        moving = " moving" if moving_match and moving_match.group(1) == "1" else ""
        if raw_pulse_match:
            print(f"S3 {mode}: {servo_pulse_us}us{duty}")
        else:
            current = current_match.group(1) if current_match else "?"
            target = target_match.group(1) if target_match else "?"
            print(f"S3 {mode}: {current}deg -> {target}deg, {servo_pulse_us}us{duty}{moving}")

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def show_twist():
        print(f"motion: vx={vx:+.2f} m/s vy={vy:+.2f} m/s wz={wz:+.2f} rad/s")

    def show_ga25():
        direction = "forward" if ga25_direction > 0 else "reverse"
        print(f"GA25: {direction}, {ga25_speed}%")

    try:
        show_s3_status()
        tty.setcbreak(sys.stdin.fileno())
        next_refresh = time.monotonic()
        while True:
            timeout = max(0, next_refresh - time.monotonic())
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == "\x1b":
                    break
                if key == " ":
                    vx = vy = wz = 0.0
                    issue("stop")
                    continue
                if key == "w":
                    vx = clamp(vx + args.linear, args.max_linear)
                    show_twist()
                elif key == "s":
                    vx = clamp(vx - args.linear, args.max_linear)
                    show_twist()
                elif key == "a":
                    vy = clamp(vy + args.linear, args.max_linear)
                    show_twist()
                elif key == "d":
                    vy = clamp(vy - args.linear, args.max_linear)
                    show_twist()
                elif key == "q":
                    wz = clamp(wz + args.yaw, args.max_yaw)
                    show_twist()
                elif key == "e":
                    wz = clamp(wz - args.yaw, args.max_yaw)
                    show_twist()
                elif key == "x":
                    vx = 0.0
                    show_twist()
                elif key == "z":
                    vy = 0.0
                    show_twist()
                elif key == "c":
                    wz = 0.0
                    show_twist()
                elif key == "[":
                    servo_pulse_us = max(800, servo_pulse_us - args.servo_pulse_step)
                    issue(f"s3 pulse {servo_pulse_us}", show=False)
                    show_s3_status()
                elif key == "]":
                    servo_pulse_us = min(2125, servo_pulse_us + args.servo_pulse_step)
                    issue(f"s3 pulse {servo_pulse_us}", show=False)
                    show_s3_status()
                elif key == "r":
                    issue("s3 release", show=False)
                    show_s3_status()
                elif key == "f":
                    ga25_direction = 1
                    show_ga25()
                elif key == "b":
                    ga25_direction = -1
                    show_ga25()
                elif key == ",":
                    ga25_speed = max(0, ga25_speed - 5)
                    show_ga25()
                elif key == ".":
                    ga25_speed = min(100, ga25_speed + 5)
                    show_ga25()
                elif key == "g":
                    ga25_speed = 0
                    issue("ga25 0", show=False)
                    show_ga25()
                elif key == "t":
                    issue("state")
                    show_s3_status()
                else: continue

            now = time.monotonic()
            if now >= next_refresh:
                issue(f"twist {vx:.3f} {vy:.3f} {wz:.3f}", show=False)
                if ga25_speed:
                    issue(f"ga25 {ga25_direction * ga25_speed}", show=False)
                next_refresh = now + args.period
    finally:
        issue("stop", show=False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
