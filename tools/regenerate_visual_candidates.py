#!/usr/bin/env python3
"""Regenerate arena visual candidates from a recorded video and telemetry log."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from render_bev_log import fence_observations, load_projector


ARENA_LENGTH = 3.0
ARENA_WIDTH = 1.985


def wall_distance(x, y):
    horizontal_x = np.clip(x, 0, ARENA_LENGTH)
    vertical_y = np.clip(y, 0, ARENA_WIDTH)
    return np.minimum.reduce((
        np.hypot(x - horizontal_x, y),
        np.hypot(x - ARENA_LENGTH, y - vertical_y),
        np.hypot(x - horizontal_x, y - ARENA_WIDTH),
        np.hypot(x, y - vertical_y),
    ))


def match(points, yaw_prior, maximum=4):
    if len(points) < 20:
        return []
    x_values = np.arange(0, ARENA_LENGTH + 1e-9, .05)
    y_values = np.arange(0, ARENA_WIDTH + 1e-9, .05)
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="ij")
    translations = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    keep = max(10, len(points) * 2 // 3)
    range_weights = 1.0 / np.square(.20 + np.linalg.norm(points, axis=1))
    evaluated = []
    for offset_deg in range(-25, 26, 5):
        yaw = yaw_prior + math.radians(offset_deg)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotated_x = cosine * points[:, 0] - sine * points[:, 1]
        rotated_y = sine * points[:, 0] + cosine * points[:, 1]
        x = translations[:, 0, None] + rotated_x[None, :]
        y = translations[:, 1, None] + rotated_y[None, :]
        distances = wall_distance(x, y)
        order = np.argpartition(distances, keep - 1, axis=1)[:, :keep]
        kept_distances = np.take_along_axis(distances, order, axis=1)
        kept_weights = range_weights[order]
        residuals = np.sum(kept_weights * kept_distances, axis=1) / np.sum(
            kept_weights, axis=1)
        best = np.argpartition(residuals, min(15, len(residuals) - 1))[:16]
        evaluated.extend((float(residuals[index]), float(translations[index, 0]),
                          float(translations[index, 1]), yaw) for index in best)
    evaluated.sort()
    candidates = []
    for residual, x, y, yaw in evaluated:
        if all(math.hypot(x - item[0], y - item[1]) >= .30 for item in candidates):
            candidates.append([x, y, (yaw + math.pi) % (2 * math.pi) - math.pi,
                               residual])
        if len(candidates) == maximum:
            break
    return candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--calibration", type=Path,
                        default=Path("config/camera_fisheye_1280x720.yaml"))
    parser.add_argument("--height", type=float, default=.1291)
    parser.add_argument("--pitch", type=float, default=30.0296)
    parser.add_argument("--roll", type=float, default=.2071)
    parser.add_argument("--fence-height", type=float, default=.254)
    parser.add_argument("--hsv-lower", type=int, nargs=3, default=(96, 128, 82))
    parser.add_argument("--hsv-upper", type=int, nargs=3, default=(121, 255, 255))
    args = parser.parse_args()
    telemetry_path = args.run_dir / "telemetry.jsonl"
    output_path = args.output or args.run_dir / "telemetry-dual-edge.jsonl"
    records = [json.loads(line) for line in telemetry_path.open()]
    capture = cv2.VideoCapture(str(args.run_dir / "video.avi"))
    if not capture.isOpened():
        raise RuntimeError("cannot open run video")
    projector = load_projector(args.calibration, (320, 180), args.height,
                               args.pitch, args.roll)
    current = []
    with output_path.open("w") as output:
        for index, record in enumerate(records):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("video ended at frame {}".format(index))
            if index % 5 == 0:
                _, lower, upper, points = fence_observations(
                    frame, projector, (320, 180), args.hsv_lower, args.hsv_upper,
                    args.fence_height)
                current = match(points, record["odometry_pose"][2])
                record["lower_fence_points"] = len(lower)
                record["upper_fence_points"] = len(upper)
            record["visual_geometry_candidates"] = current
            output.write(json.dumps(record, separators=(",", ":")) + "\n")
            if index and index % 250 == 0:
                print("processed {}/{}".format(index, len(records)), flush=True)
    capture.release()
    print("wrote {} frames to {}".format(len(records), output_path))


if __name__ == "__main__":
    main()
