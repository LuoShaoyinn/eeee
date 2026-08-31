#!/usr/bin/env python3
"""Keyboard teleoperation through the local robotd Unix socket."""

import argparse
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
                        help="per-axis velocity increment in m/s")
    parser.add_argument("--yaw", type=float, default=.50,
                        help="yaw-rate increment in rad/s")
    parser.add_argument("--period", type=float, default=.08,
                        help="twist refresh period in seconds")
    parser.add_argument("--servo-step", type=int, default=5)
    args = parser.parse_args()
    if args.linear <= 0 or args.yaw <= 0 or args.period <= 0 or args.servo_step <= 0:
        parser.error("all control increments and period must be positive")

    print("W/S forward/back, A/D left/right, Q/E yaw, Space stop, Esc quit")
    print("[ / ] S3 angle, R releases S3, , / . GA25 duty, G stops GA25, T state")
    vx = vy = wz = 0.0
    servo_angle = 0
    ga25_duty = 0
    old_settings = termios.tcgetattr(sys.stdin)

    def issue(command, show=True):
        try:
            reply = request(args.socket, command)
            if show:
                print(reply)
        except OSError as error:
            print(f"robotd error: {error}", file=sys.stderr)

    try:
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
                if key == "w": vx = args.linear
                elif key == "s": vx = -args.linear
                elif key == "a": vy = args.linear
                elif key == "d": vy = -args.linear
                elif key == "q": wz = args.yaw
                elif key == "e": wz = -args.yaw
                elif key == "x": vx = 0.0
                elif key == "z": vy = 0.0
                elif key == "c": wz = 0.0
                elif key == "[":
                    servo_angle = max(0, servo_angle - args.servo_step)
                    issue(f"s3 {servo_angle}")
                elif key == "]":
                    servo_angle = min(120, servo_angle + args.servo_step)
                    issue(f"s3 {servo_angle}")
                elif key == "r": issue("s3 release")
                elif key == ",": ga25_duty = max(-100, ga25_duty - 5)
                elif key == ".": ga25_duty = min(100, ga25_duty + 5)
                elif key == "g": ga25_duty = 0
                elif key == "t": issue("state")
                else: continue
                print(f"twist {vx:+.2f} {vy:+.2f} {wz:+.2f}; ga25 {ga25_duty:+d}%")

            now = time.monotonic()
            if now >= next_refresh:
                issue(f"twist {vx:.3f} {vy:.3f} {wz:.3f}", show=False)
                if ga25_duty:
                    issue(f"ga25 {ga25_duty}", show=False)
                next_refresh = now + args.period
    finally:
        issue("stop", show=False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
