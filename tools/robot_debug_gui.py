#!/usr/bin/env python3
"""Host-side arena viewer with manual and explicitly started automatic control."""

import argparse
import getpass
import json
import math
import os
import queue
import socket
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path


class DebugGui:
    def __init__(self, root, args, ssh_client, control_stdin, control_stdout):
        self.root = root
        self.args = args
        self.ssh_client = ssh_client
        self.control_stdin = control_stdin
        self.control_stdout = control_stdout
        self.events = queue.Queue()
        self.stopping = threading.Event()
        self.auto_running = False
        self.vx = self.vy = self.wz = 0.0
        self.pose = None
        self.trail = []
        self.odometry_trail = []

        root.title("Robot arena debug")
        root.geometry("1000x720")
        toolbar = tk.Frame(root)
        toolbar.pack(fill=tk.X, padx=8, pady=8)
        self.auto_button = tk.Button(toolbar, text="Start Auto", width=10,
                                     command=self.start_auto, bg="#b7d7b0")
        self.auto_button.pack(side=tk.LEFT)
        tk.Button(toolbar, text="STOP", width=10, command=self.stop_motion,
                  bg="#e8a4a4").pack(side=tk.LEFT, padx=(8, 0))
        tk.Button(toolbar, text="Clear trail", command=self.clear_trail).pack(side=tk.LEFT, padx=8)
        self.motion_label = tk.Label(toolbar, anchor="w")
        self.motion_label.pack(side=tk.LEFT, padx=16)
        self.status_label = tk.Label(toolbar, anchor="e")
        self.status_label.pack(side=tk.RIGHT)
        self.canvas = tk.Canvas(root, bg="#f4f5f2", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.footer = tk.Label(
            root,
            text="W/S forward  A/D left  Q/E turn  X/Z/C clear axes  Space stop | "
                 "S3 operational 1600-2000 us (not controlled here)",
            anchor="w")
        self.footer.pack(fill=tk.X, padx=8, pady=(0, 8))

        root.bind("<KeyPress>", self.on_key)
        root.bind("<Configure>", lambda _event: self.draw())
        root.protocol("WM_DELETE_WINDOW", self.close)
        threading.Thread(target=self.receive_udp, daemon=True).start()
        threading.Thread(target=self.read_control_replies, daemon=True).start()
        self.update_motion_label()
        self.root.after(50, self.poll_events)
        self.root.after(args.command_period_ms, self.refresh_command)

    def send(self, command):
        try:
            self.control_stdin.write(command + "\n")
            self.control_stdin.flush()
        except Exception as error:
            self.events.put(("error", "control channel: {}".format(error)))
            self.auto_running = False

    def start_auto(self):
        self.vx = self.vy = self.wz = 0.0
        self.send("stop")
        self.auto_running = True
        self.auto_button.configure(text="Auto Running", bg="#edc46f")
        self.update_motion_label()

    def stop_motion(self):
        self.vx = self.vy = self.wz = 0.0
        self.auto_running = False
        self.send("stop")
        self.auto_button.configure(text="Start Auto", bg="#b7d7b0")
        self.update_motion_label()

    def clear_trail(self):
        self.trail.clear()
        self.odometry_trail.clear()
        self.draw()

    def on_key(self, event):
        key = event.keysym.lower()
        if key == "space":
            self.stop_motion()
            return "break"
        if key in ("w", "s", "a", "d", "q", "e", "x", "z", "c"):
            self.auto_running = False
            self.auto_button.configure(text="Start Auto", bg="#b7d7b0")
        if key == "w": self.vx = min(self.args.max_linear, self.vx + self.args.linear_step)
        elif key == "s": self.vx = max(-self.args.max_linear, self.vx - self.args.linear_step)
        elif key == "a": self.vy = min(self.args.max_linear, self.vy + self.args.linear_step)
        elif key == "d": self.vy = max(-self.args.max_linear, self.vy - self.args.linear_step)
        elif key == "q": self.wz = min(self.args.max_yaw, self.wz + self.args.yaw_step)
        elif key == "e": self.wz = max(-self.args.max_yaw, self.wz - self.args.yaw_step)
        elif key == "x": self.vx = 0.0
        elif key == "z": self.vy = 0.0
        elif key == "c": self.wz = 0.0
        else: return None
        self.update_motion_label()
        return "break"

    def refresh_command(self):
        if self.auto_running and self.pose:
            proposal = self.pose.get("auto_proposal", {})
            twist = proposal.get("twist", [0, 0, 0])
            if proposal.get("valid", False) and not proposal.get("reached", False):
                self.send("twist {:.3f} {:.3f} {:.3f}".format(*twist))
            else:
                self.send("stop")
        elif any(abs(value) > 1e-6 for value in (self.vx, self.vy, self.wz)):
            self.send("twist {:.3f} {:.3f} {:.3f}".format(self.vx, self.vy, self.wz))
        if not self.stopping.is_set():
            self.root.after(self.args.command_period_ms, self.refresh_command)

    def update_motion_label(self):
        state = "AUTO" if self.auto_running else "MANUAL"
        self.motion_label.configure(
            text="{}   vx={:+.2f}  vy={:+.2f} m/s  wz={:+.2f} rad/s".format(
                state, self.vx, self.vy, self.wz))

    def receive_udp(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind(("", self.args.port))
        receiver.settimeout(0.5)
        while not self.stopping.is_set():
            try:
                payload, source = receiver.recvfrom(65535)
                record = json.loads(payload)
                self.events.put(("pose", record, source[0]))
            except socket.timeout:
                continue
            except (OSError, ValueError) as error:
                if not self.stopping.is_set():
                    self.events.put(("error", "UDP: {}".format(error)))
        receiver.close()

    def read_control_replies(self):
        try:
            for line in self.control_stdout:
                self.events.put(("reply", line.strip()))
        except Exception as error:
            if not self.stopping.is_set():
                self.events.put(("error", "SSH control: {}".format(error)))

    def poll_events(self):
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "pose":
                    self.pose = event[1]
                    self.args.log_file.write(json.dumps(self.pose, separators=(",", ":")) + "\n")
                    self.args.log_file.flush()
                    point = tuple(self.pose.get("pose", [0, 0, 0])[:2])
                    if not self.trail or math.dist(point, self.trail[-1]) > 0.005:
                        self.trail.append(point)
                        self.trail = self.trail[-2000:]
                    odometry = tuple(self.pose.get("odometry_pose", [0, 0, 0])[:2])
                    if not self.odometry_trail or math.dist(odometry, self.odometry_trail[-1]) > 0.005:
                        self.odometry_trail.append(odometry)
                        self.odometry_trail = self.odometry_trail[-2000:]
                    age = self.pose.get("telemetry_age_ms", -1)
                    valid = self.pose.get("telemetry_valid", False)
                    geometry = self.pose.get("visual_geometry", {})
                    axis_certainty = geometry.get("axis_certainty", [0, 0, 0])
                    proposal = self.pose.get("auto_proposal", {})
                    self.status_label.configure(
                        text="{}  frame {}  UART {} {:.0f} ms  visual X/Y/yaw {:.0%}/{:.0%}/{:.0%}  {} {:.1f}s".format(
                            event[2], self.pose.get("frame_index", "?"),
                            "OK" if valid else "STALE", age,
                            *axis_certainty,
                            proposal.get("phase", "tracking"),
                            proposal.get("lost_seconds", 0)),
                        fg="#176b2c" if valid else "#a02b2b")
                    self.draw()
                elif event[0] == "error":
                    self.status_label.configure(text=event[1], fg="#a02b2b")
                    self.auto_running = False
                    self.update_motion_label()
        except queue.Empty:
            pass
        if not self.stopping.is_set():
            self.root.after(50, self.poll_events)

    def transform(self, x_m, y_m):
        width = max(self.canvas.winfo_width(), 100)
        height = max(self.canvas.winfo_height(), 100)
        margin = 55
        scale = min((width - 2 * margin) / self.args.arena_length,
                    (height - 2 * margin) / self.args.arena_width)
        left = (width - self.args.arena_length * scale) / 2
        bottom = (height + self.args.arena_width * scale) / 2
        return left + x_m * scale, bottom - y_m * scale, scale

    def draw(self):
        self.canvas.delete("all")
        x0, y0, scale = self.transform(0, 0)
        x1, y1, _ = self.transform(self.args.arena_length, self.args.arena_width)
        self.canvas.create_rectangle(x0, y1, x1, y0, fill="#e8e9e5", outline="#245ca6", width=5)
        hx0, hy0, _ = self.transform(0, 0)
        hx1, hy1, _ = self.transform(.2, .3)
        self.canvas.create_polygon(hx0, hy0, hx1, hy0, hx1, hy1, hx0, hy1,
                                   fill="#363b40", outline="#111")
        ox0, oy0, _ = self.transform(self.args.arena_length - .2, self.args.arena_width - .3)
        ox1, oy1, _ = self.transform(self.args.arena_length, self.args.arena_width)
        self.canvas.create_rectangle(ox0, oy1, ox1, oy0, fill="#b9bec2", outline="#555")
        self.canvas.create_text(x0, y0 + 24, text="home (0,0)", anchor="w")
        if len(self.trail) > 1:
            coordinates = []
            for point in self.trail:
                px, py, _ = self.transform(*point)
                coordinates.extend((px, py))
            self.canvas.create_line(*coordinates, fill="#d36f24", width=2)
        if len(self.odometry_trail) > 1:
            coordinates = []
            for point in self.odometry_trail:
                px, py, _ = self.transform(*point)
                coordinates.extend((px, py))
            self.canvas.create_line(*coordinates, fill="#2676c9", width=2, dash=(5, 3))
        if not self.pose:
            self.canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                    text="Waiting for UDP location on port {}".format(self.args.port))
            return
        x_m, y_m, yaw = self.pose.get("pose", [0, 0, 0])
        px, py, _ = self.transform(x_m, y_m)
        sigma = max(0.0, float(self.pose.get("position_sigma_m", 0))) * scale
        self.canvas.create_oval(px - sigma, py - sigma, px + sigma, py + sigma,
                                outline="#cc3f3f", width=2)
        radius = 9
        self.canvas.create_oval(px - radius, py - radius, px + radius, py + radius,
                                fill="#1d7f57", outline="#083d2b", width=2)
        heading = 34
        self.canvas.create_line(px, py, px + heading * math.cos(yaw),
                                py - heading * math.sin(yaw), fill="#111", width=3,
                                arrow=tk.LAST)
        self.canvas.create_text(px + 12, py - 16,
                                text="PF ({:.2f}, {:.2f})  {:.1f} deg  sigma {:.2f} m".format(
                                    x_m, y_m, math.degrees(yaw), sigma / scale), anchor="w")
        odom_x, odom_y, odom_yaw = self.pose.get("odometry_pose", [0, 0, 0])
        opx, opy, _ = self.transform(odom_x, odom_y)
        self.canvas.create_oval(opx - 6, opy - 6, opx + 6, opy + 6,
                                fill="#2676c9", outline="#174a7d")
        self.canvas.create_line(opx, opy, opx + 26 * math.cos(odom_yaw),
                                opy - 26 * math.sin(odom_yaw), fill="#174a7d", width=2,
                                arrow=tk.LAST)
        self.canvas.create_text(opx + 10, opy + 14,
                                text="wheel+IMU ({:.2f}, {:.2f})".format(odom_x, odom_y),
                                fill="#174a7d", anchor="w")
        home = self.pose.get("home_box", {})
        if home.get("detected", False):
            home_x, home_y = home.get("position", [0, 0])
            hpx, hpy, _ = self.transform(home_x, home_y)
            colour = "#238b45" if home.get("consistent", False) else "#c7352b"
            self.canvas.create_rectangle(hpx - 7, hpy - 7, hpx + 7, hpy + 7,
                                         outline=colour, width=3)
            self.canvas.create_text(
                hpx + 10, hpy + 12,
                text="home box {:.2f} m".format(home.get("error_m", 0)),
                fill=colour, anchor="w")
        candidates = self.pose.get("visual_geometry_candidates", [])
        geometry = self.pose.get("visual_geometry", {})
        colours = ("#8b2db3", "#b15bc7", "#ca8ed9", "#dfb9e8")
        for index, candidate in enumerate(candidates):
            if len(candidate) < 4:
                continue
            candidate_x, candidate_y, candidate_yaw, residual = candidate
            cpx, cpy, _ = self.transform(candidate_x, candidate_y)
            colour = colours[min(index, len(colours) - 1)]
            size = 10 if index == 0 else 7
            self.canvas.create_line(cpx - size, cpy, cpx + size, cpy, fill=colour, width=3)
            self.canvas.create_line(cpx, cpy - size, cpx, cpy + size, fill=colour, width=3)
            self.canvas.create_line(cpx, cpy, cpx + 22 * math.cos(candidate_yaw),
                                    cpy - 22 * math.sin(candidate_yaw), fill=colour, width=2)
            self.canvas.create_text(cpx + 9, cpy - 12,
                                    text="V{} {:.3f}m".format(index + 1, residual),
                                    fill=colour, anchor="w")
        if candidates and geometry.get("valid", False):
            center_x, center_y = candidates[0][:2]
            major = 2.0 * geometry.get("sigma_major_m", 0)
            minor = 2.0 * geometry.get("sigma_minor_m", 0)
            axis = geometry.get("major_axis_rad", 0)
            ellipse = []
            for step in range(49):
                angle = 2 * math.pi * step / 48
                local_x = major * math.cos(angle)
                local_y = minor * math.sin(angle)
                world_x = center_x + math.cos(axis) * local_x - math.sin(axis) * local_y
                world_y = center_y + math.sin(axis) * local_x + math.cos(axis) * local_y
                pixel_x, pixel_y, _ = self.transform(world_x, world_y)
                ellipse.extend((pixel_x, pixel_y))
            self.canvas.create_line(*ellipse, fill="#8b2db3", width=2)
            confidence = geometry.get("confidence", 0)
            axis_certainty = geometry.get("axis_certainty", [0, 0, 0])
            margin = geometry.get("alternative_margin_m", 0)
            self.canvas.create_text(
                16, 18,
                text="visual X/Y/yaw {:.0%}/{:.0%}/{:.0%} | legacy {:.0%} | 2sigma {:.2f} x {:.2f} m | gap {:.3f} m".format(
                    *axis_certainty, confidence, major, minor, margin),
                fill="#5f197b", anchor="nw")
        object_colours = ("#d5a000", "#d3342f", "#333333", "#666666")
        object_names = ("cylinder", "cube", "home", "opponent")
        for item in self.pose.get("objects", []):
            if len(item) < 4: continue
            class_id, object_x, object_y, confidence = item
            opx, opy, _ = self.transform(object_x, object_y)
            colour = object_colours[int(class_id)]
            self.canvas.create_oval(opx - 7, opy - 7, opx + 7, opy + 7,
                                    fill=colour, outline="#111")
            self.canvas.create_text(opx + 10, opy - 10,
                                    text="{} {:.0%}".format(object_names[int(class_id)], confidence),
                                    fill=colour, anchor="w")

    def close(self):
        self.stopping.set()
        self.send("stop")
        try:
            self.control_stdin.close()
        finally:
            self.args.log_file.close()
            self.ssh_client.close()
            self.root.destroy()


def connect_control(args):
    try:
        import paramiko
    except ImportError as error:
        raise RuntimeError("install tools dependencies with: uv sync --project tools") from error
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    options = dict(hostname=args.host, username=args.user, timeout=10,
                   auth_timeout=10, banner_timeout=10)
    if args.identity_file.is_file():
        options.update(key_filename=str(args.identity_file), look_for_keys=False, allow_agent=False)
    else:
        password = os.environ.get("CUBIE_PASSWORD") or getpass.getpass("Cubie SSH password: ")
        options.update(password=password, look_for_keys=False, allow_agent=False)
    client.connect(**options)
    command = "cd {} && exec ./build/robotctl --stream".format(args.remote_dir)
    stdin, stdout, stderr = client.exec_command(command, get_pty=False)
    if stdout.channel.exit_status_ready():
        raise RuntimeError(stderr.read().decode(errors="replace"))
    return client, stdin, stdout


def main():
    parser = argparse.ArgumentParser(description="View UDP localization and control the robot over SSH.")
    parser.add_argument("--host", default="192.168.19.105")
    parser.add_argument("--user", default="radxa")
    parser.add_argument("--identity-file", type=Path,
                        default=Path.home() / ".ssh/id_ed25519_cubie_192_168_19_105")
    parser.add_argument("--remote-dir", default="/home/radxa/cubie-robot")
    parser.add_argument("--port", type=int, default=3335)
    parser.add_argument("--arena-length", type=float, default=3.0)
    parser.add_argument("--arena-width", type=float, default=1.985)
    parser.add_argument("--linear-step", type=float, default=.05)
    parser.add_argument("--yaw-step", type=float, default=.25)
    parser.add_argument("--max-linear", type=float, default=.45)
    parser.add_argument("--max-yaw", type=float, default=2.0)
    parser.add_argument("--command-period-ms", type=int, default=80)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    if args.log is None:
        args.log = Path("run-log") / ("gui-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".jsonl")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log_file = args.log.open("w")
    try:
        client, stdin, stdout = connect_control(args)
    except Exception as error:
        print("connection failed: {}".format(error))
        return 1
    root = tk.Tk()
    DebugGui(root, args, client, stdin, stdout)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
