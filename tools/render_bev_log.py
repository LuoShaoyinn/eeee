#!/usr/bin/env python3
"""Render synchronized camera, fence projection, and arena localization diagnostics."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


ARENA_LENGTH_M = 3.0
ARENA_WIDTH_M = 1.985
OUTPUT_SIZE = (1280, 720)
CAMERA_VIEW_SIZE = (640, 360)


def load_projector(calibration_path, visual_size, height_m, pitch_deg, roll_deg):
    storage = cv2.FileStorage(str(calibration_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise RuntimeError("cannot open calibration: {}".format(calibration_path))
    matrix = storage.getNode("rectified_K").mat()
    image_width = storage.getNode("image_width").real()
    image_height = storage.getNode("image_height").real()
    storage.release()
    if matrix is None:
        raise RuntimeError("calibration has no rectified_K")
    width, height = visual_size
    matrix = matrix.astype(np.float64)
    matrix[0, :] *= width / image_width
    matrix[1, :] *= height / image_height
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    pitch_rotation = np.array([
        [0, -math.sin(pitch), math.cos(pitch)],
        [-1, 0, 0],
        [0, -math.cos(pitch), -math.sin(pitch)],
    ])
    roll_rotation = np.array([
        [math.cos(roll), -math.sin(roll), 0],
        [math.sin(roll), math.cos(roll), 0],
        [0, 0, 1],
    ])
    return np.linalg.inv(matrix), pitch_rotation @ roll_rotation, height_m


def fence_observations(frame, projector, visual_size):
    inverse, rotation, camera_height = projector
    small = cv2.resize(frame, visual_size, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (92, 75, 45), (135, 255, 255))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3)))
    pixels = []
    ground = []
    for x in range(0, visual_size[0], 4):
        rows = np.flatnonzero(mask[visual_size[1] // 5:, x])
        if not len(rows):
            continue
        y = int(rows[-1] + visual_size[1] // 5)
        ray = rotation @ (inverse @ np.array([x, y, 1.0]))
        if ray[2] >= -1e-6:
            continue
        point = (-camera_height / ray[2]) * ray
        if -0.5 < point[0] < 6 and abs(point[1]) < 4:
            pixels.append((x, y))
            ground.append(point[:2])
    return small, pixels, np.asarray(ground, dtype=np.float64)


def transform_points(points, pose):
    if not len(points):
        return points
    x_m, y_m, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.array([x_m, y_m])


class ArenaCanvas:
    def __init__(self):
        self.left = 680
        self.top = 55
        self.scale = min(560 / ARENA_LENGTH_M, 400 / ARENA_WIDTH_M)
        self.bottom = self.top + ARENA_WIDTH_M * self.scale

    def point(self, position):
        return (int(round(self.left + position[0] * self.scale)),
                int(round(self.bottom - position[1] * self.scale)))

    def draw_arena(self, image):
        top_left = self.point((0, ARENA_WIDTH_M))
        bottom_right = self.point((ARENA_LENGTH_M, 0))
        cv2.rectangle(image, top_left, bottom_right, (225, 228, 224), -1)
        cv2.rectangle(image, top_left, bottom_right, (166, 92, 36), 5)
        cv2.rectangle(image, self.point((0, .3)), self.point((.2, 0)), (55, 55, 55), -1)
        cv2.rectangle(image, self.point((2.8, 1.985)), self.point((3, 1.685)),
                      (170, 170, 170), -1)
        cv2.putText(image, "home", (self.left, int(round(self.bottom + 25))),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (50, 50, 50), 1, cv2.LINE_AA)

    def draw_trail(self, image, trail, colour):
        if len(trail) > 1:
            cv2.polylines(image, [np.asarray([self.point(p) for p in trail], np.int32)],
                          False, colour, 2, cv2.LINE_AA)

    def draw_pose(self, image, pose, colour, label, radius=8):
        center = self.point(pose[:2])
        cv2.circle(image, center, radius, colour, -1, cv2.LINE_AA)
        endpoint = (int(center[0] + 28 * math.cos(pose[2])),
                    int(center[1] - 28 * math.sin(pose[2])))
        cv2.arrowedLine(image, center, endpoint, colour, 2, cv2.LINE_AA, tipLength=.25)
        cv2.putText(image, label, (center[0] + 9, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv2.LINE_AA)

    def draw_cloud(self, image, points, colour):
        for point in points:
            pixel = self.point(point)
            if -10 <= pixel[0] < OUTPUT_SIZE[0] + 10 and -10 <= pixel[1] < OUTPUT_SIZE[1] + 10:
                cv2.circle(image, pixel, 2, colour, -1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--calibration", type=Path,
                        default=Path("config/camera_fisheye_1280x720.yaml"))
    parser.add_argument("--height", type=float, default=.1291)
    parser.add_argument("--pitch", type=float, default=30.0296)
    parser.add_argument("--roll", type=float, default=.2071)
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--replay", type=Path,
                        help="optional offline PF JSONL to overlay")
    args = parser.parse_args()
    telemetry_path = args.run_dir / "telemetry.jsonl"
    video_path = args.run_dir / "video.avi"
    output_path = args.output or args.run_dir / "bev-debug.avi"
    records = [json.loads(line) for line in telemetry_path.open()]
    replay = ([json.loads(line) for line in args.replay.open()]
              if args.replay else None)
    if replay is not None and len(replay) != len(records):
        raise RuntimeError("replay and telemetry frame counts differ")
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError("cannot open {}".format(video_path))
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"MJPG"),
                             args.fps, OUTPUT_SIZE)
    if not writer.isOpened():
        raise RuntimeError("cannot create {}".format(output_path))
    visual_size = (320, 180)
    projector = load_projector(args.calibration, visual_size, args.height,
                               args.pitch, args.roll)
    arena = ArenaCanvas()
    pf_trail = []
    odometry_trail = []
    replay_trail = []
    count = 0
    try:
        for record in records:
            ok, frame = capture.read()
            if not ok:
                break
            small, edge_pixels, observations = fence_observations(frame, projector, visual_size)
            camera_view = cv2.resize(small, CAMERA_VIEW_SIZE, interpolation=cv2.INTER_LINEAR)
            for x, y in edge_pixels:
                cv2.circle(camera_view, (x * 2, y * 2), 3, (0, 165, 255), -1, cv2.LINE_AA)
            output = np.full((OUTPUT_SIZE[1], OUTPUT_SIZE[0], 3), 246, np.uint8)
            output[:CAMERA_VIEW_SIZE[1], :CAMERA_VIEW_SIZE[0]] = camera_view
            arena.draw_arena(output)
            pf_pose = record["pose"]
            odometry_pose = record.get("odometry_pose", pf_pose)
            pf_trail.append(pf_pose[:2])
            odometry_trail.append(odometry_pose[:2])
            arena.draw_trail(output, pf_trail, (40, 125, 220))
            arena.draw_trail(output, odometry_trail, (200, 105, 35))
            arena.draw_cloud(output, transform_points(observations, pf_pose), (60, 160, 235))
            candidates = record.get("visual_geometry_candidates", [])
            if candidates:
                arena.draw_cloud(output, transform_points(observations, candidates[0][:3]),
                                 (175, 60, 160))
            sigma_px = int(record.get("position_sigma_m", 0) * arena.scale)
            if sigma_px:
                cv2.circle(output, arena.point(pf_pose[:2]), sigma_px, (60, 60, 200), 1, cv2.LINE_AA)
            arena.draw_pose(output, pf_pose, (25, 120, 65), "PF")
            arena.draw_pose(output, odometry_pose, (200, 95, 30), "wheel+IMU", 6)
            if replay is not None:
                replay_pose = replay[count]["pose"]
                replay_trail.append(replay_pose[:2])
                arena.draw_trail(output, replay_trail, (190, 80, 190))
                arena.draw_pose(output, replay_pose, (190, 80, 190), "offline PF", 7)
            for index, candidate in enumerate(candidates):
                center = arena.point(candidate[:2])
                colour = (175, 60 + index * 25, 160)
                cv2.drawMarker(output, center, colour, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
                cv2.putText(output, "V{} {:.3f}m".format(index + 1, candidate[3]),
                            (center[0] + 7, center[1] + 14), cv2.FONT_HERSHEY_SIMPLEX,
                            .38, colour, 1, cv2.LINE_AA)
            cv2.putText(output, "rectified camera + extracted lower fence edge", (12, 385),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (35, 35, 35), 1, cv2.LINE_AA)
            lines = [
                "frame {}   t={:.1f}s   UART {} age {:.0f}ms".format(
                    record["frame_index"], count / args.fps,
                    "OK" if record["telemetry_valid"] else "STALE",
                    record["telemetry_age_ms"]),
                "PF       x={:.2f} y={:.2f} yaw={:.1f}deg sigma={:.3f}m".format(
                    pf_pose[0], pf_pose[1], math.degrees(pf_pose[2]),
                    record.get("position_sigma_m", 0)),
                "wheel+IMU x={:.2f} y={:.2f} yaw={:.1f}deg".format(
                    odometry_pose[0], odometry_pose[1], math.degrees(odometry_pose[2])),
                "targets  " + " ".join("{:+.2f}".format(value) for value in record["targets"]),
                "purple cloud: lower fence projected using V1; orange cloud: using PF",
            ]
            if replay is not None:
                lines[-1] = "offline PF x={:.2f} y={:.2f} yaw={:.1f}deg visual={}".format(
                    replay_pose[0], replay_pose[1], math.degrees(replay_pose[2]),
                    "accepted" if replay[count]["visual_accepted"] else "prediction")
            for index, line in enumerate(lines):
                cv2.putText(output, line, (20, 440 + index * 38), cv2.FONT_HERSHEY_SIMPLEX,
                            .55, (35, 35, 35), 1, cv2.LINE_AA)
            writer.write(output)
            count += 1
            if count % 250 == 0:
                print("rendered {}/{}".format(count, len(records)), flush=True)
    finally:
        capture.release()
        writer.release()
    print("wrote {} frames to {}".format(count, output_path))


if __name__ == "__main__":
    main()
