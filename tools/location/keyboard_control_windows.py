#!/usr/bin/env python3
"""Windows keyboard teleoperation through an SSH tunnel to local robotd."""

import argparse
import getpass
import msvcrt
import os
import re
import shlex
import subprocess
import sys
import time


# Runs on the Cubie. Keeping the Unix-socket client on the Cubie means robotd
# remains private to the board and continues to be the sole UART owner.
BRIDGE_PROGRAM = r"""
import socket
import sys

socket_path = sys.argv[1]
for wire in sys.stdin:
    command = wire.strip()
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(0.5)
            client.connect(socket_path)
            client.sendall((command + "\n").encode())
            reply = client.recv(1024).decode(errors="replace").strip()
        print(reply, flush=True)
    except OSError as error:
        print("robotd error: " + str(error), flush=True)
"""


def make_remote_command(remote_python, socket_path):
    return "{} -u -c {} {}".format(
        shlex.quote(remote_python),
        shlex.quote(BRIDGE_PROGRAM),
        shlex.quote(socket_path),
    )


def open_openssh_bridge(args):
    destination = args.host
    if args.user:
        destination = "{}@{}".format(args.user, destination)
    command = [
        args.ssh,
        "-T",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout={}".format(args.connect_timeout),
        destination,
        make_remote_command(args.remote_python, args.socket),
    ]
    try:
        return subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
    except FileNotFoundError as error:
        raise RuntimeError(
            "cannot start {!r}; install Windows OpenSSH Client or use --ssh".format(args.ssh)
        ) from error


class ParamikoBridge:
    def __init__(self, client, channel):
        self.client = client
        self.channel = channel
        self.writer = channel.makefile("w")
        self.reader = channel.makefile("r")

    def request(self, command):
        if self.channel.closed:
            raise RuntimeError("SSH bridge closed")
        try:
            self.writer.write(command + "\n")
            self.writer.flush()
            reply = self.reader.readline()
        except OSError as error:
            raise RuntimeError("SSH bridge I/O failed: {}".format(error)) from error
        if not reply:
            raise RuntimeError("SSH bridge closed without replying")
        return reply.rstrip("\r\n")

    def close(self):
        self.writer.close()
        self.reader.close()
        self.channel.close()
        self.client.close()


def open_password_bridge(args):
    try:
        import paramiko
    except ImportError as error:
        raise RuntimeError(
            "password authentication needs Paramiko; run: python -m pip install paramiko"
        ) from error

    password = os.environ.get(args.password_env)
    if password is None:
        password = getpass.getpass("Cubie SSH password: ")
    if not password:
        raise RuntimeError("empty SSH password")

    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(
            hostname=args.host,
            username=args.user,
            password=password,
            timeout=args.connect_timeout,
            auth_timeout=args.connect_timeout,
            banner_timeout=args.connect_timeout,
            look_for_keys=False,
            allow_agent=False,
        )
        channel = client.get_transport().open_session()
        channel.settimeout(args.connect_timeout)
        channel.exec_command(make_remote_command(args.remote_python, args.socket))
        return ParamikoBridge(client, channel)
    except Exception as error:
        client.close()
        raise RuntimeError("password SSH connection failed: {}".format(error)) from error


def open_bridge(args):
    if args.auth == "password":
        return open_password_bridge(args)
    return open_openssh_bridge(args)


def request(bridge, command):
    if isinstance(bridge, ParamikoBridge):
        return bridge.request(command)
    if bridge.poll() is not None:
        raise RuntimeError("SSH bridge exited with code {}".format(bridge.returncode))
    try:
        bridge.stdin.write(command + "\n")
        bridge.stdin.flush()
        reply = bridge.stdout.readline()
    except (BrokenPipeError, OSError) as error:
        raise RuntimeError("SSH bridge I/O failed: {}".format(error)) from error
    if not reply:
        raise RuntimeError("SSH bridge closed without replying")
    return reply.rstrip("\r\n")


def close_bridge(bridge):
    if isinstance(bridge, ParamikoBridge):
        bridge.close()
        return
    bridge.terminate()
    try:
        bridge.wait(timeout=2)
    except subprocess.TimeoutExpired:
        bridge.kill()


def read_key():
    key = msvcrt.getwch()
    # Consume Windows extended-key sequences (arrows, function keys, etc.).
    if key in ("\x00", "\xe0"):
        msvcrt.getwch()
        return ""
    return key.lower()


def main():
    parser = argparse.ArgumentParser(
        description="Windows keyboard teleoperation via SSH to Cubie robotd"
    )
    parser.add_argument("--host", required=True, help="Cubie IP address or SSH host alias")
    parser.add_argument("--user", default="radxa", help="SSH username (default: radxa)")
    parser.add_argument("--auth", choices=("key", "password"), default="key",
                        help="SSH authentication method (default: key)")
    parser.add_argument("--password-env", default="CUBIE_SSH_PASSWORD",
                        help="password environment variable for --auth password")
    parser.add_argument("--ssh", default="ssh", help="path to ssh.exe (default: ssh)")
    parser.add_argument("--remote-python", default="python3",
                        help="Python command available on the Cubie")
    parser.add_argument("--socket", default="/tmp/robotd.sock",
                        help="robotd Unix socket path on the Cubie")
    parser.add_argument("--connect-timeout", type=int, default=10,
                        help="SSH connection timeout in seconds")
    parser.add_argument("--linear", type=float, default=.10,
                        help="linear velocity increment per key press in m/s")
    parser.add_argument("--yaw", type=float, default=.50,
                        help="yaw-rate increment per key press in rad/s")
    parser.add_argument("--max-linear", type=float, default=.60,
                        help="maximum absolute linear velocity in m/s")
    parser.add_argument("--max-yaw", type=float, default=2.00,
                        help="maximum absolute yaw rate in rad/s")
    parser.add_argument("--period", type=float, default=.08,
                        help="twist refresh period in seconds")
    parser.add_argument("--servo-pulse-step", type=int, default=25,
                        help="S3 raw-pulse increment in microseconds")
    args = parser.parse_args()
    if (args.connect_timeout <= 0 or args.linear <= 0 or args.yaw <= 0 or
            args.max_linear <= 0 or args.max_yaw <= 0 or args.period <= 0 or
            args.servo_pulse_step <= 0):
        parser.error("all timeouts, control increments, and period must be positive")

    try:
        bridge = open_bridge(args)
    except RuntimeError as error:
        print("SSH error: {}".format(error), file=sys.stderr)
        return 1

    vx = vy = wz = 0.0
    servo_pulse_us = 1500
    ga25_speed = 0
    ga25_direction = 1

    def issue(command, show=True):
        try:
            reply = request(bridge, command)
        except RuntimeError as error:
            print("SSH error: {}".format(error), file=sys.stderr)
            return None
        if reply.startswith(("robotd error:", "error:")):
            print(reply, file=sys.stderr)
            return None
        if show:
            print(reply)
        return reply

    def show_s3_status():
        nonlocal servo_pulse_us
        reply = issue("s3", show=False)
        if reply is None:
            return False
        raw_pulse_match = re.search(r"raw (\d+)us", reply)
        current_pulse_match = re.search(r"current \d+deg (\d+)us", reply)
        pulse_match = raw_pulse_match or current_pulse_match
        if pulse_match is None:
            print(reply)
            return True
        servo_pulse_us = int(pulse_match.group(1))
        mode_match = re.search(r"^s3 (\w+)", reply)
        current_match = re.search(r"current (\d+)deg", reply)
        target_match = re.search(r"target (\d+)deg", reply)
        duty_match = re.search(r"(\d+\.\d+)%", reply)
        moving_match = re.search(r"moving (\d+)", reply)
        mode = mode_match.group(1) if mode_match else "unknown"
        duty = ", {}%".format(duty_match.group(1)) if duty_match else ""
        moving = " moving" if moving_match and moving_match.group(1) == "1" else ""
        if raw_pulse_match:
            print("S3 {}: {}us{}".format(mode, servo_pulse_us, duty))
        else:
            current = current_match.group(1) if current_match else "?"
            target = target_match.group(1) if target_match else "?"
            print("S3 {}: {}deg -> {}deg, {}us{}{}".format(
                mode, current, target, servo_pulse_us, duty, moving))
        return True

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def show_twist():
        print("motion: vx={:+.2f} m/s vy={:+.2f} m/s wz={:+.2f} rad/s".format(vx, vy, wz))

    def show_ga25():
        direction = "forward" if ga25_direction > 0 else "reverse"
        print("GA25: {}, {}%".format(direction, ga25_speed))

    try:
        if issue("state") is None:
            return 1
        if not show_s3_status():
            return 1
        print("Connected through SSH to {}.".format(args.host))
        print("W/S forward/back, A/D left/right, Q/E yaw: each press adds speed")
        print("Space stop, Esc quit; X/Z/C clear one motion axis")
        print("[ / ] S3 pulse, R releases S3; F/B direction, ,/. GA25 speed, G stops GA25, T state")
        print("limits: linear +-{:.2f} m/s, yaw +-{:.2f} rad/s".format(
            args.max_linear, args.max_yaw))
        next_refresh = time.monotonic()
        while True:
            if msvcrt.kbhit():
                key = read_key()
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
                    issue("s3 pulse {}".format(servo_pulse_us), show=False)
                    if not show_s3_status():
                        break
                elif key == "]":
                    servo_pulse_us = min(2125, servo_pulse_us + args.servo_pulse_step)
                    issue("s3 pulse {}".format(servo_pulse_us), show=False)
                    if not show_s3_status():
                        break
                elif key == "r":
                    issue("s3 release", show=False)
                    if not show_s3_status():
                        break
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
                    if issue("state") is None or not show_s3_status():
                        break

            now = time.monotonic()
            if now >= next_refresh:
                if issue("twist {:.3f} {:.3f} {:.3f}".format(vx, vy, wz), show=False) is None:
                    break
                if ga25_speed and issue(
                        "ga25 {}".format(ga25_direction * ga25_speed), show=False) is None:
                    break
                next_refresh = now + args.period
            time.sleep(.005)
    except KeyboardInterrupt:
        pass
    finally:
        issue("stop", show=False)
        close_bridge(bridge)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
