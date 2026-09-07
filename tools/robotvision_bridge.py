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
    """Project a rectified pixel to chassis or collector-centred ground plane.

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

    def _project(self, pixel_x: float, pixel_y: float, reference_forward_m: float,
                 reference_left_m: float):
        ray = self.rotation @ (self.inverse @ np.array((pixel_x, pixel_y, 1.0)))
        if ray[2] >= -1e-6:
            return None
        scale = -self.height_m / ray[2]
        forward_m = scale * ray[0] + self.camera_forward_m - reference_forward_m
        left_m = scale * ray[1] + self.camera_left_m - reference_left_m
        if not (math.isfinite(forward_m) and math.isfinite(left_m) and .02 < forward_m < 6.0 and abs(left_m) < 4.0):
            return None
        return forward_m, left_m

    def project(self, pixel_x: float, pixel_y: float):
        """Return a point relative to the collector centreline for visual servo."""
        return self._project(pixel_x, pixel_y, self.collector_forward_m, self.collector_left_m)

    def project_chassis(self, pixel_x: float, pixel_y: float):
        """Return a point relative to the chassis origin for field localization."""
        return self._project(pixel_x, pixel_y, 0.0, 0.0)


def blue_fence_ground_points(blue_mask: np.ndarray, projector: GroundProjector, step: int = 8,
                             lower_row_fraction: float = .30):
    """Sample the lower blue-fence envelope and project it onto the field plane."""
    height, width = blue_mask.shape[:2]
    points = []
    for pixel_x in range(0, width, step):
        rows = np.flatnonzero(blue_mask[:, pixel_x])
        rows = rows[rows >= int(height * lower_row_fraction)]
        if rows.size == 0:
            continue
        ground = projector.project_chassis(float(pixel_x), float(rows[-1]))
        if ground is not None:
            points.append(ground)
    return points


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


class BlueFenceParticleFilter:
    """Wheel/IMU prediction plus blue-fence ground-plane particle correction."""
    def __init__(self, count: int, arena_length_m: float, arena_width_m: float, x_m: float, y_m: float,
                 yaw_deg: float, measurement_sigma_m: float, seed: int):
        self.count = count
        self.arena_length_m = arena_length_m
        self.arena_width_m = arena_width_m
        self.measurement_sigma_m = measurement_sigma_m
        self.rng = np.random.default_rng(seed)
        self.particles = np.zeros((count, 3), dtype=np.float64)
        self.weights = np.full(count, 1.0 / count, dtype=np.float64)
        self.last_time = time.monotonic()
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = math.radians(yaw_deg)
        self.effective_particles = float(count)
        self.last_fence_update = False
        self.last_fence_update_time = None
        self.reset(x_m, y_m, yaw_deg)

    def _refresh_estimate(self) -> None:
        self.x_m = float(np.dot(self.weights, self.particles[:, 0]))
        self.y_m = float(np.dot(self.weights, self.particles[:, 1]))
        self.yaw_rad = math.atan2(float(np.dot(self.weights, np.sin(self.particles[:, 2]))),
                                  float(np.dot(self.weights, np.cos(self.particles[:, 2]))))
        self.effective_particles = float(1.0 / np.dot(self.weights, self.weights))

    def predict(self, forward_mps: float, left_mps: float, yaw_radps: float) -> None:
        now = time.monotonic()
        dt = min(.25, max(0.0, now - self.last_time))
        self.last_time = now
        if dt == 0.0:
            return
        forward = forward_mps * dt + self.rng.normal(0.0, .002 + .04 * abs(forward_mps * dt), self.count)
        left = left_mps * dt + self.rng.normal(0.0, .002 + .04 * abs(left_mps * dt), self.count)
        yaw = yaw_radps * dt + self.rng.normal(0.0, .002 + .035 * abs(yaw_radps * dt), self.count)
        cosine, sine = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        self.particles[:, 0] = np.clip(self.particles[:, 0] + cosine * forward - sine * left,
                                       0.0, self.arena_length_m)
        self.particles[:, 1] = np.clip(self.particles[:, 1] + sine * forward + cosine * left,
                                       0.0, self.arena_width_m)
        self.particles[:, 2] = (self.particles[:, 2] + yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._refresh_estimate()

    def update_fence(self, observations) -> bool:
        if len(observations) < 20:
            self.last_fence_update = False
            return False
        points = np.asarray(observations[:120], dtype=np.float64)
        cosine, sine = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        global_x = self.particles[:, 0, None] + cosine[:, None] * points[None, :, 0] - sine[:, None] * points[None, :, 1]
        global_y = self.particles[:, 1, None] + sine[:, None] * points[None, :, 0] + cosine[:, None] * points[None, :, 1]
        residuals = np.minimum.reduce((np.abs(global_x), np.abs(self.arena_length_m - global_x),
                                       np.abs(global_y), np.abs(self.arena_width_m - global_y)))
        keep = max(12, int(residuals.shape[1] * 2 / 3))
        trimmed_mean = np.partition(residuals, keep - 1, axis=1)[:, :keep].mean(axis=1)
        likelihood = np.exp(-.5 * np.square(trimmed_mean / self.measurement_sigma_m))
        self.weights *= np.maximum(likelihood, 1e-12)
        normalizer = float(self.weights.sum())
        if normalizer <= 1e-20:
            self.weights.fill(1.0 / self.count)
        else:
            self.weights /= normalizer
        self._refresh_estimate()
        if self.effective_particles < self.count * .55:
            positions = (self.rng.random() + np.arange(self.count)) / self.count
            indices = np.searchsorted(np.cumsum(self.weights), positions, side="right")
            self.particles = self.particles[np.minimum(indices, self.count - 1)]
            self.weights.fill(1.0 / self.count)
            self._refresh_estimate()
        self.last_fence_update = True
        self.last_fence_update_time = time.monotonic()
        return True

    def reset(self, x_m: float, y_m: float, yaw_deg: float) -> None:
        self.particles[:, 0] = np.clip(self.rng.normal(x_m, .06, self.count), 0.0, self.arena_length_m)
        self.particles[:, 1] = np.clip(self.rng.normal(y_m, .06, self.count), 0.0, self.arena_width_m)
        self.particles[:, 2] = (self.rng.normal(math.radians(yaw_deg), math.radians(8.0), self.count) + math.pi) % \
                               (2.0 * math.pi) - math.pi
        self.weights.fill(1.0 / self.count)
        self.last_time = time.monotonic()
        self.last_fence_update = False
        self.last_fence_update_time = None
        self._refresh_estimate()

    def render(self, path: str, localization_valid: bool, blue_pixels: int, fence_points: int) -> None:
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
        cv2.putText(canvas, "V1 PARTICLE-FILTER POSITION", (40, 32), cv2.FONT_HERSHEY_SIMPLEX, .68, (235, 235, 235), 2)
        cv2.putText(canvas, status, (40, 62), cv2.FONT_HERSHEY_SIMPLEX, .60, color, 2)
        cv2.putText(canvas, f"x={self.x_m:.2f}m  y={self.y_m:.2f}m  yaw={math.degrees(self.yaw_rad):.0f}deg",
                    (40, height - 12), cv2.FONT_HERSHEY_SIMPLEX, .57, (235, 235, 235), 2)
        cv2.putText(canvas, f"blue: {blue_pixels}  ground: {fence_points}", (405, 62), cv2.FONT_HERSHEY_SIMPLEX, .42,
                    (210, 210, 210), 1)
        cv2.putText(canvas, f"ESS: {self.effective_particles:.0f}/{self.count}", (40, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, (210, 210, 210), 1)
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
    parser.add_argument("--collector-left-m", type=float, default=-.02,
                        help="calibrated intake centre: 0.02 m to the camera's right (left is positive)")
    parser.add_argument("--particle-count", type=int, default=600)
    parser.add_argument("--particle-seed", type=int, default=1)
    parser.add_argument("--fence-measurement-sigma-m", type=float, default=.07)
    parser.add_argument("--fence-hold-seconds", type=float, default=1.0,
                        help="keep the last valid blue-fence PF correction briefly across sparse frames")
    parser.add_argument("--fence-lower-row-fraction", type=float, default=.30)
    parser.add_argument("--arena-length-m", type=float, default=3.0)
    parser.add_argument("--arena-width-m", type=float, default=1.985)
    args = parser.parse_args()
    if (args.particle_count < 50 or args.fence_measurement_sigma_m <= 0.0 or
            not 0.0 < args.fence_lower_row_fraction < 1.0 or args.arena_length_m <= 0.0 or
            args.arena_width_m <= 0.0 or not 0.0 <= args.initial_x <= args.arena_length_m or
            not 0.0 <= args.initial_y <= args.arena_width_m):
        raise SystemExit("particle-filter parameters or initial pose are outside the arena")

    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 20)
    if not capture.isOpened():
        raise SystemExit(f"cannot open camera: {args.camera}")
    maps = None
    projector = None
    frames = 0
    pose = BlueFenceParticleFilter(args.particle_count, args.arena_length_m, args.arena_width_m,
                                   args.initial_x, args.initial_y, args.initial_yaw,
                                   args.fence_measurement_sigma_m, args.particle_seed)
    reset_timestamp_ns = None
    encoder_forward_m = 0.0
    encoder_last_time = time.monotonic()
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
            blue_mask = cv2.inRange(hsv, (92, 75, 45), (135, 255, 255))
            blue_pixels = int(cv2.countNonZero(blue_mask))
            blue_detected = blue_pixels >= args.minimum_blue_pixels
            fence_points = blue_fence_ground_points(blue_mask, projector,
                                                     lower_row_fraction=args.fence_lower_row_fraction) if projector else []
            try:
                reset_stat = os.stat(args.localization_reset_file)
                if reset_stat.st_mtime_ns != reset_timestamp_ns:
                    with open(args.localization_reset_file, encoding="utf-8") as reset_input:
                        reset = json.load(reset_input)
                    x_m, y_m, yaw_deg = float(reset["x_m"]), float(reset["y_m"]), float(reset["yaw_deg"])
                    if not 0.0 <= x_m <= args.arena_length_m or not 0.0 <= y_m <= args.arena_width_m:
                        raise ValueError("reset pose is outside the arena")
                    pose.reset(x_m, y_m, yaw_deg)
                    reset_timestamp_ns = reset_stat.st_mtime_ns
            except FileNotFoundError:
                pass
            try:
                motion = parse_motion(robotd_request(args.robotd_socket, "state"),
                                      robotd_request(args.robotd_socket, "telemetry"))
                now = time.monotonic()
                encoder_forward_m += motion[0] * min(.25, max(0.0, now - encoder_last_time))
                encoder_last_time = now
                pose.predict(*motion)
            except (OSError, ValueError, StopIteration):
                # Keep the last broad pose visible. The motion watchdog in
                # robotd remains authoritative if telemetry is unavailable.
                pass
            fence_updated = pose.update_fence(fence_points) if blue_detected else False
            # A blue-pixel count alone is not sufficient for autonomous
            # motion: require enough projectable lower-fence points to make a
            # particle-filter measurement update in this frame.
            fence_recent = pose.last_fence_update_time is not None and \
                           time.monotonic() - pose.last_fence_update_time <= args.fence_hold_seconds
            localization_valid = fence_recent
            pose.render(args.localization_map, localization_valid, blue_pixels, len(fence_points))
            write_json(args.localization_file, {
                "valid": localization_valid,
                "x_m": round(pose.x_m, 3),
                "y_m": round(pose.y_m, 3),
                "yaw_deg": round(math.degrees(pose.yaw_rad), 1),
                "blue_pixels": blue_pixels,
                "fence_ground_points": len(fence_points),
                "effective_particles": round(pose.effective_particles, 1),
                "fence_update": fence_updated,
                "encoder_forward_m": round(encoder_forward_m, 3),
                "odometry_source": "wheel_encoder_rpm_fused_with_blue_fence_pf_gate",
                "source": "v1_blue_fence_particle_filter",
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
            # ODOM is a signed cumulative wheel-encoder distance.  The
            # mission records its value at intake entry and closes a 0.25 m
            # local run with it; autonomous motion remains gated by the
            # concurrently updated blue-fence particle filter.
            frame = ["1" if localization_valid else "0", "0", "ODOM", f"{encoder_forward_m:.3f}"]
            # PF pose is chassis-centred; Home target is the camera ground position.
            c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
            camera_x = pose.x_m + c * args.camera_forward_m - s * args.camera_left_m
            camera_y = pose.y_m + s * args.camera_forward_m + c * args.camera_left_m
            frame += ["POSE", f"{camera_x:.4f}", f"{camera_y:.4f}", f"{pose.yaw_rad:.5f}"]
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
