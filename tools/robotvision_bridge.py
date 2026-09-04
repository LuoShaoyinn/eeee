#!/usr/bin/env python3
"""Emit calibrated A733 YOLO detections in robotbrain's frame protocol.

This bridge keeps camera correction explicit while reusing the already
validated A733 YOLO26 executable.  It is intentionally a supervised V1 path:
it never commands motors.  Pipe its output to robotbrain only after checking
the emitted frames first.
"""

import argparse
import json
import math
import os
import re
import socket
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np


LABELS = {
    "yellow_cylinder": "yellow",
    "red_cube": "red",
    "other_robot": "other_robot",
    "home": "home",
}
DETECTION = re.compile(
    r"^(yellow_cylinder|red_cube|other_robot|home)\s+([0-9.]+)%\s+\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]$")


def load_maps(path: str, width: int, height: int):
    calibration = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not calibration.isOpened():
        raise RuntimeError(f"cannot open calibration: {path}")
    source_k = calibration.getNode("K").mat()
    distortion = calibration.getNode("D").mat()
    rectified_k = calibration.getNode("rectified_K").mat()
    calibration.release()
    if source_k is None or distortion is None or rectified_k is None:
        raise RuntimeError("calibration must contain K, D, and rectified_K")
    source_k = source_k.astype(np.float64)
    distortion = distortion.astype(np.float64)
    rectified_k = rectified_k.astype(np.float64)
    scale_x, scale_y = width / 1280.0, height / 720.0
    for matrix in (source_k, rectified_k):
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
    maps = cv2.fisheye.initUndistortRectifyMap(
        source_k, distortion, np.eye(3), rectified_k, (width, height), cv2.CV_16SC2)
    return maps, rectified_k


class GroundProjector:
    """Project a rectified pixel to the collector-centred ground plane.

    The angular terms come from the physical camera mount calibration.  The
    two camera translation values are deliberately separate command-line
    settings: optical calibration alone cannot tell us where a left-mounted
    camera sits relative to the collector intake.
    """
    def __init__(self, camera_matrix: np.ndarray, height_m: float, pitch_down_deg: float,
                 roll_deg: float, camera_forward_m: float, camera_left_m: float,
                 collector_forward_m: float, collector_left_m: float):
        self.inverse = np.linalg.inv(camera_matrix)
        pitch = math.radians(pitch_down_deg)
        roll = math.radians(roll_deg)
        pitch_rotation = np.array(((0.0, -math.sin(pitch), math.cos(pitch)),
                                   (-1.0, 0.0, 0.0),
                                   (0.0, -math.cos(pitch), -math.sin(pitch))), dtype=np.float64)
        roll_rotation = np.array(((math.cos(roll), -math.sin(roll), 0.0),
                                  (math.sin(roll), math.cos(roll), 0.0),
                                  (0.0, 0.0, 1.0)), dtype=np.float64)
        self.rotation = pitch_rotation @ roll_rotation
        self.height_m = height_m
        self.camera_forward_m = camera_forward_m
        self.camera_left_m = camera_left_m
        self.collector_forward_m = collector_forward_m
        self.collector_left_m = collector_left_m

    def project(self, pixel_x: float, pixel_y: float):
        ray = self.rotation @ (self.inverse @ np.array((pixel_x, pixel_y, 1.0)))
        if ray[2] >= -1e-6:
            return None
        scale = -self.height_m / ray[2]
        forward_m = scale * ray[0] + self.camera_forward_m - self.collector_forward_m
        left_m = scale * ray[1] + self.camera_left_m - self.collector_left_m
        if not (math.isfinite(forward_m) and math.isfinite(left_m) and .02 < forward_m < 6.0 and abs(left_m) < 4.0):
            return None
        return forward_m, left_m


def parse_detections(output: str, width: int, height: int):
    detections = []
    for line in output.splitlines():
        match = DETECTION.match(line.strip())
        if not match:
            continue
        label, score, left, top, right, bottom = match.groups()
        left, top, right, bottom = map(int, (left, top, right, bottom))
        detections.append((LABELS[label], float(score) / 100.0, left, top, right, bottom))
    return detections


def write_protocol_frame(path: str, frame: str) -> None:
    """Atomically publish the newest V1 frame for the live mission runner."""
    directory = os.path.dirname(path) or "."
    descriptor, temporary_path = tempfile.mkstemp(prefix=".robotvision-", dir=directory, text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            output.write(frame + "\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass
        raise


def write_json(path: str, payload: dict) -> None:
    directory = os.path.dirname(path) or "."
    descriptor, temporary_path = tempfile.mkstemp(prefix=".robotpose-", dir=directory, text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(payload, output)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass
        raise


def robotd_request(socket_path: str, command: str) -> str:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(.15)
        client.connect(socket_path)
        client.sendall((command + "\n").encode())
        return client.recv(1024).decode(errors="replace")


def parse_motion(state_reply: str, telemetry_reply: str):
    words = iter(state_reply.replace(";", " ").split())
    gyro_z = 0.0
    rpm = [0.0] * 4
    for word in words:
        if word == "gyro":
            next(words), next(words)
            gyro_z = float(next(words))
        elif word == "rpm":
            rpm = [float(next(words)) for _ in range(4)]
    tokens = telemetry_reply.replace(";", " ").split()
    targets = [0.0] * 4
    if "target" in tokens:
        index = tokens.index("target") + 1
        targets = [float(value) for value in tokens[index:index + 4]]
    wheel = []
    for measured, target in zip(rpm, targets):
        direction = 1.0 if target > .03 else -1.0 if target < -.03 else 0.0
        wheel.append(direction * measured * 2.0 * math.pi * .023 / 60.0)
    return ((wheel[0] + wheel[1] + wheel[2] + wheel[3]) / 4.0,
            (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) / 4.0,
            -gyro_z * math.pi / 180.0)


class V1PoseTracker:
    """Broad arena pose for the operator map; blue fence validates each update."""
    def __init__(self, x_m: float, y_m: float, yaw_deg: float):
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = math.radians(yaw_deg)
        self.last_time = time.monotonic()

    def update(self, forward_mps: float, left_mps: float, yaw_radps: float) -> None:
        now = time.monotonic()
        dt = min(.2, max(0.0, now - self.last_time))
        self.last_time = now
        self.x_m = min(3.0, max(0.0, self.x_m + (math.cos(self.yaw_rad) * forward_mps -
                                                   math.sin(self.yaw_rad) * left_mps) * dt))
        self.y_m = min(1.985, max(0.0, self.y_m + (math.sin(self.yaw_rad) * forward_mps +
                                                    math.cos(self.yaw_rad) * left_mps) * dt))
        self.yaw_rad = (self.yaw_rad + yaw_radps * dt + math.pi) % (2.0 * math.pi) - math.pi

    def reset(self, x_m: float, y_m: float, yaw_deg: float) -> None:
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = math.radians(yaw_deg)
        self.last_time = time.monotonic()

    def render(self, path: str, localization_valid: bool, blue_pixels: int) -> None:
        width, height, margin = 720, 500, 40
        canvas = np.full((height, width, 3), (22, 30, 34), dtype=np.uint8)
        scale = min((width - 2 * margin) / 3.0, (height - 2 * margin) / 1.985)
        field_w, field_h = int(3.0 * scale), int(1.985 * scale)
        origin = (margin, height - margin)
        cv2.rectangle(canvas, (origin[0], origin[1] - field_h), (origin[0] + field_w, origin[1]),
                      (245, 170, 45), 3)
        for meter in (1, 2):
            x = origin[0] + int(meter * scale)
            cv2.line(canvas, (x, origin[1] - field_h), (x, origin[1]), (55, 66, 72), 1)
        cv2.line(canvas, (origin[0], origin[1] - int(scale)), (origin[0] + field_w, origin[1] - int(scale)),
                 (55, 66, 72), 1)
        px = origin[0] + int(self.x_m * scale)
        py = origin[1] - int(self.y_m * scale)
        radius = 14
        cv2.circle(canvas, (px, py), radius, (70, 220, 105) if localization_valid else (50, 150, 235), -1)
        tip = (px + int(30 * math.cos(self.yaw_rad)), py - int(30 * math.sin(self.yaw_rad)))
        cv2.arrowedLine(canvas, (px, py), tip, (245, 245, 245), 3, tipLength=.32)
        status = "BLUE FENCE: VALID" if localization_valid else "BLUE FENCE: LOST"
        color = (70, 220, 105) if localization_valid else (50, 150, 235)
        cv2.putText(canvas, "V1 ARENA POSITION", (40, 32), cv2.FONT_HERSHEY_SIMPLEX, .78, (235, 235, 235), 2)
        cv2.putText(canvas, status, (40, 62), cv2.FONT_HERSHEY_SIMPLEX, .60, color, 2)
        cv2.putText(canvas, f"x={self.x_m:.2f}m  y={self.y_m:.2f}m  yaw={math.degrees(self.yaw_rad):.0f}deg",
                    (40, height - 12), cv2.FONT_HERSHEY_SIMPLEX, .57, (235, 235, 235), 2)
        cv2.putText(canvas, f"blue pixels: {blue_pixels}", (430, 62), cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (210, 210, 210), 1)
        temporary = path + ".tmp.png"
        if not cv2.imwrite(temporary, canvas):
            raise RuntimeError(f"cannot write localization map: {path}")
        os.replace(temporary, path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--calibration", default="config/camera_fisheye_1280x720.yaml")
    parser.add_argument("--yolo-dir", default="/home/radxa/yolo26n-a733")
    parser.add_argument("--yolo-bin", default="./yolo26_demo_640_a733",
                        help="A733 executable matching the selected model's decoder")
    parser.add_argument("--model", default="/home/radxa/cubie-robot/models/official_yolo26n_640x384_rgbfix_rebuild_a733.nb",
                        help="RGB-corrected 640x384 A733 INT8 model with two 4x5040 outputs")
    parser.add_argument("--minimum-blue-pixels", type=int, default=60)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means stream until interrupted")
    parser.add_argument("--frame-path", default="/tmp/robotvision-rectified.jpg")
    parser.add_argument("--protocol-file", default="/tmp/robotvision-frame.txt",
                        help="atomically updated latest frame for the supervised mission runner")
    parser.add_argument("--robotd-socket", default="/tmp/robotd.sock")
    parser.add_argument("--localization-file", default="/tmp/robot-localization.json")
    parser.add_argument("--localization-map", default="/tmp/robot-localization-map.png")
    parser.add_argument("--initial-x", type=float, default=.10)
    parser.add_argument("--initial-y", type=float, default=.10)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--localization-reset-file", default="/tmp/robot-localization-reset.json")
    parser.add_argument("--camera-height-m", type=float, default=.1311825723,
                        help="optical-centre height from camera1_mount calibration")
    parser.add_argument("--camera-pitch-deg", type=float, default=30.11324982,
                        help="positive-down optical-axis pitch from camera1_mount calibration")
    parser.add_argument("--camera-roll-deg", type=float, default=.2071)
    parser.add_argument("--camera-forward-m", type=float, default=0.0,
                        help="camera optical centre forward of the chassis origin (measure this)")
    parser.add_argument("--camera-left-m", type=float, default=0.0,
                        help="camera optical centre left of the chassis origin (measure this; left is positive)")
    parser.add_argument("--collector-forward-m", type=float, default=0.0,
                        help="intake centre forward of the chassis origin (measure this)")
    parser.add_argument("--collector-left-m", type=float, default=0.0,
                        help="intake centre left of the chassis origin (usually zero)")
    args = parser.parse_args()
    if not 0.0 <= args.initial_x <= 3.0 or not 0.0 <= args.initial_y <= 1.985:
        raise SystemExit("initial pose must lie inside the 3.0m x 1.985m arena")

    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 20)
    if not capture.isOpened():
        raise SystemExit(f"cannot open camera: {args.camera}")
    maps = None
    projector = None
    frames = 0
    pose = V1PoseTracker(args.initial_x, args.initial_y, args.initial_yaw)
    reset_timestamp_ns = None
    try:
        while not args.max_frames or frames < args.max_frames:
            ok, raw = capture.read()
            if not ok or raw is None:
                raise RuntimeError("camera capture failed")
            if maps is None:
                maps, rectified_k = load_maps(args.calibration, raw.shape[1], raw.shape[0])
                projector = GroundProjector(rectified_k, args.camera_height_m, args.camera_pitch_deg,
                                            args.camera_roll_deg, args.camera_forward_m, args.camera_left_m,
                                            args.collector_forward_m, args.collector_left_m)
            rectified = cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR)
            hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
            blue_pixels = int(cv2.countNonZero(cv2.inRange(hsv, (92, 75, 45), (135, 255, 255))))
            localization_valid = blue_pixels >= args.minimum_blue_pixels
            try:
                reset_stat = os.stat(args.localization_reset_file)
                if reset_stat.st_mtime_ns != reset_timestamp_ns:
                    with open(args.localization_reset_file, encoding="utf-8") as reset_input:
                        reset = json.load(reset_input)
                    x_m, y_m, yaw_deg = float(reset["x_m"]), float(reset["y_m"]), float(reset["yaw_deg"])
                    if not 0.0 <= x_m <= 3.0 or not 0.0 <= y_m <= 1.985:
                        raise ValueError("reset pose is outside the arena")
                    pose.reset(x_m, y_m, yaw_deg)
                    reset_timestamp_ns = reset_stat.st_mtime_ns
            except FileNotFoundError:
                pass
            try:
                pose.update(*parse_motion(robotd_request(args.robotd_socket, "state"),
                                          robotd_request(args.robotd_socket, "telemetry")))
            except (OSError, ValueError, StopIteration):
                # Keep the last broad pose visible. The motion watchdog in
                # robotd remains authoritative if telemetry is unavailable.
                pass
            pose.render(args.localization_map, localization_valid, blue_pixels)
            write_json(args.localization_file, {
                "valid": localization_valid,
                "x_m": round(pose.x_m, 3),
                "y_m": round(pose.y_m, 3),
                "yaw_deg": round(math.degrees(pose.yaw_rad), 1),
                "blue_pixels": blue_pixels,
                "source": "v1_blue_fence_wheel_imu",
            })
            environment = os.environ.copy()
            environment["LD_LIBRARY_PATH"] = args.yolo_dir + ":" + environment.get("LD_LIBRARY_PATH", "")
            # The NPU runner accepts a file path.  Write the corrected, unannotated
            # image first: otherwise it would consume the preceding annotated frame.
            inference_path = args.frame_path + ".inference.jpg"
            if not cv2.imwrite(inference_path, rectified):
                raise RuntimeError(f"cannot write corrected inference frame: {inference_path}")
            result = subprocess.run(
                [args.yolo_bin, "-nb", args.model, "-i", inference_path, "-l", "1"],
                cwd=args.yolo_dir, env=environment, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False)
            if result.returncode != 0:
                raise RuntimeError("A733 YOLO failed: " + result.stdout[-500:])
            detections = parse_detections(result.stdout, rectified.shape[1], rectified.shape[0])
            annotated = rectified.copy()
            frame = ["1" if localization_valid else "0", "0"]
            ground_log = []
            for label, confidence, left, top, right, bottom in detections:
                center_x = max(0.0, min(1.0, (left + right) * .5 / rectified.shape[1]))
                bottom_y = max(0.0, min(1.0, bottom / rectified.shape[0]))
                frame.extend((label, f"{confidence:.3f}", f"{center_x:.3f}", f"{bottom_y:.3f}"))
                # The detection's bottom centre is the point most likely to
                # touch the arena floor.  Send it in addition to legacy image
                # coordinates, so robotbrain can fall back safely for old logs.
                ground = projector.project((left + right) * .5, float(bottom)) if projector else None
                if ground is not None:
                    frame.extend(("@", f"{ground[0]:.3f}", f"{ground[1]:.3f}"))
                    ground_log.append(f"{label} forward={ground[0]:.3f}m left={ground[1]:+.3f}m")
                else:
                    ground_log.append(f"{label} ground=invalid")
                cv2.rectangle(annotated, (left, top), (right, bottom), (255, 170, 0), 2)
                annotation = f"{label} {confidence:.0%}"
                if ground is not None:
                    annotation += f" {ground[0]:.2f}m/{ground[1]:+.2f}m"
                cv2.putText(annotated, annotation, (left, max(18, top - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 170, 0), 2)
            temporary_path = args.frame_path + ".tmp.jpg"
            if not cv2.imwrite(temporary_path, annotated):
                raise RuntimeError(f"cannot write rectified frame: {temporary_path}")
            os.replace(temporary_path, args.frame_path)
            protocol_frame = " ".join(frame)
            write_protocol_frame(args.protocol_file, protocol_frame)
            print(protocol_frame, flush=True)
            print(f"robotvision: frame={frames} blue={blue_pixels} detections={len(detections)}",
                  file=sys.stderr, flush=True)
            if ground_log:
                print("robotvision ground: " + "; ".join(ground_log), file=sys.stderr, flush=True)
            frames += 1
    finally:
        capture.release()


if __name__ == "__main__":
    main()
