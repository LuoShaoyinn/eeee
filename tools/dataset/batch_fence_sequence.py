#!/usr/bin/env python3
"""Batch-match three observed fence panels to the alternating field perimeter."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import (CAMERA_MATRIX, FENCE_HEIGHT_M, FIELD_LENGTH_M,
                                 FIELD_WIDTH_M, extract_corner_pairs, line_distance,
                                 pose_to_extrinsics, refine_pose)
from hsv_thresholds import load_thresholds


VERTICES = np.array([[0.0, 0.0], [FIELD_LENGTH_M, 0.0],
                     [FIELD_LENGTH_M, FIELD_WIDTH_M], [0.0, FIELD_WIDTH_M]], dtype=np.float64)


def projected_visible(points, pose, height_m, pitch_down_deg, image_width=1280, image_height=720):
    rotation_vector, translation = pose_to_extrinsics(pose, height_m, pitch_down_deg)
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_points = (rotation @ points.T + translation).T
    pixels = (camera_points[:, :2] / camera_points[:, 2:3]) @ CAMERA_MATRIX[:2, :2].T
    pixels += CAMERA_MATRIX[:2, 2]
    visible = ((camera_points[:, 2] > .02) & (pixels[:, 0] >= 0) & (pixels[:, 0] < image_width) &
               (pixels[:, 1] >= 0) & (pixels[:, 1] < image_height))
    return pixels[visible]


def sequence(start, direction):
    return [(start + direction * index) % 4 for index in range(4)]


def object_corners(start, direction):
    order = sequence(start, direction)
    first, second = VERTICES[order[1]], VERTICES[order[2]]
    return np.array([[*first, 0.0], [*first, FENCE_HEIGHT_M],
                     [*second, 0.0], [*second, FENCE_HEIGHT_M]], dtype=np.float64)


def line_rms(pose, start, direction, upper_lines, lower_lines, height_m, pitch_down_deg):
    order = sequence(start, direction)
    residuals = []
    for index in range(3):
        xy = np.linspace(VERTICES[order[index]], VERTICES[order[index + 1]], 100)
        lower = projected_visible(np.column_stack((xy, np.zeros(len(xy)))), pose, height_m, pitch_down_deg)
        upper = projected_visible(np.column_stack((xy, np.full(len(xy), FENCE_HEIGHT_M))), pose, height_m, pitch_down_deg)
        # An off-screen or behind-camera part has no image measurement and must not score.
        if len(lower) >= 8:
            residuals.extend(line_distance(lower, lower_lines[index]))
        if len(upper) >= 8:
            residuals.extend(line_distance(upper, upper_lines[index]))
    if not residuals:
        return float("inf")
    return float(np.sqrt(np.mean(np.square(residuals))))


def fit_assignment(upper, lower, upper_lines, lower_lines, start, direction, height_m, pitch_down_deg):
    image_corners = np.array([lower[0], upper[0], lower[1], upper[1]], dtype=np.float64)
    objects = object_corners(start, direction)
    best = None
    for x in (.2, 1.5, 2.8):
        for y in (.15, FIELD_WIDTH_M / 2, FIELD_WIDTH_M - .15):
            for yaw in (-.7, 0.0, .7):
                pose, corner_rms = refine_pose(objects, image_corners, np.array([x, y, yaw]),
                                                height_m, pitch_down_deg)
                if not (0 < pose[0] < FIELD_LENGTH_M and 0 < pose[1] < FIELD_WIDTH_M):
                    continue
                score = line_rms(pose, start, direction, upper_lines, lower_lines,
                                 height_m, pitch_down_deg)
                candidate = (corner_rms + .2 * score, corner_rms, score, pose)
                if best is None or candidate[0] < best[0]:
                    best = candidate
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--include", default="*.jpg")
    parser.add_argument("--limit", type=int, default=20)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    args = parser.parse_args()
    thresholds = load_thresholds(args.thresholds)
    for path in sorted(args.images.glob(args.include))[:args.limit]:
        image = cv2.imread(str(path))
        try:
            _, upper, lower, upper_lines, lower_lines = extract_corner_pairs(
                image, thresholds["blue_fence"], maximum=3)
            candidates = []
            for start in range(4):
                for direction in (-1, 1):
                    result = fit_assignment(upper, lower, upper_lines, lower_lines, start, direction,
                                            .1311825723, 30.1132498)
                    if result is not None and np.isfinite(result[0]):
                        candidates.append((result, start, direction))
            if not candidates:
                raise ValueError("no valid perimeter assignment")
            (score, corner_rms, line_error, pose), start, direction = min(candidates, key=lambda item: item[0][0])
            labels = "".join("ABDC"[index] for index in sequence(start, direction))
            print("{} sequence={} score={:.2f} corner={:.2f} line={:.2f} pose={:.3f},{:.3f},{:.1f}".format(
                path.stem[-5:], labels, score, corner_rms, line_error,
                pose[0], pose[1], math.degrees(pose[2])))
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            print("{} FAIL {}".format(path.stem[-5:], error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
