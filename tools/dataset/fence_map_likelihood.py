#!/usr/bin/env python3
"""Match all four known fence walls to HSV-derived upper/lower edge distance fields.

This is an offline reference implementation.  It deliberately avoids inferred
2D corners: a candidate pose is scored only by where the complete 3D field map
projects onto the measured fence contours.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import (CAMERA_MATRIX, FENCE_HEIGHT_M, FIELD_LENGTH_M,
                                 FIELD_WIDTH_M, pose_to_extrinsics)
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def observed_distance_fields(image: np.ndarray, blue_ranges):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = make_mask(hsv, blue_ranges)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    blue = retain_fence_components(blue)
    _, _, _, upper, lower = column_edges(blue, min_pixels=8)
    return blue, upper, lower, cv2.distanceTransform(255 - upper, cv2.DIST_L2, 3), \
        cv2.distanceTransform(255 - lower, cv2.DIST_L2, 3)


def wall_points(samples_per_wall: int):
    vertices = np.array([[0.0, 0.0], [FIELD_LENGTH_M, 0.0],
                         [FIELD_LENGTH_M, FIELD_WIDTH_M], [0.0, FIELD_WIDTH_M]], float)
    xy = np.vstack([np.linspace(vertices[index], vertices[(index + 1) % 4], samples_per_wall)
                    for index in range(4)])
    return (np.column_stack((xy, np.zeros(len(xy)))),
            np.column_stack((xy, np.full(len(xy), FENCE_HEIGHT_M))))


def project_visible(points: np.ndarray, pose: np.ndarray, height_m: float, pitch_deg: float,
                    image_shape: tuple[int, int]):
    rotation_vector, translation = pose_to_extrinsics(pose, height_m, pitch_deg)
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera = points @ rotation.T + translation.reshape(1, 3)
    depth = camera[:, 2]
    projected = camera[:, :2] / depth[:, None]
    projected = projected @ CAMERA_MATRIX[:2, :2].T + CAMERA_MATRIX[:2, 2]
    height, width = image_shape
    visible = ((depth > .05) & (projected[:, 0] >= 0) & (projected[:, 0] < width) &
               (projected[:, 1] >= 0) & (projected[:, 1] < height))
    return projected, visible


def robust_distance(distance: np.ndarray, pixels: np.ndarray, valid: np.ndarray):
    x = np.clip(np.rint(pixels[valid, 0]).astype(int), 0, distance.shape[1] - 1)
    y = np.clip(np.rint(pixels[valid, 1]).astype(int), 0, distance.shape[0] - 1)
    values = np.minimum(distance[y, x], 16.0)
    if len(values) < 30:
        return None
    # Fence portions can be genuinely outside FOV or occluded.  Retain the
    # strongest 70% support rather than forcing every map sample to match.
    return float(np.mean(np.partition(values, int(len(values) * .70))[:int(len(values) * .70)]))


def score_pose(pose, lower_points, upper_points, lower_dt, upper_dt, height_m, pitch_deg):
    lower_px, lower_valid = project_visible(lower_points, pose, height_m, pitch_deg, lower_dt.shape)
    upper_px, upper_valid = project_visible(upper_points, pose, height_m, pitch_deg, upper_dt.shape)
    lower_score = robust_distance(lower_dt, lower_px, lower_valid)
    upper_score = robust_distance(upper_dt, upper_px, upper_valid)
    if lower_score is None or upper_score is None:
        return math.inf
    return lower_score + upper_score


def candidate_grid(center, span_xy, yaw_span_deg, step_m, yaw_step_deg):
    xs = np.arange(center[0] - span_xy, center[0] + span_xy + step_m / 2, step_m)
    ys = np.arange(center[1] - span_xy, center[1] + span_xy + step_m / 2, step_m)
    yaws = np.arange(center[2] - math.radians(yaw_span_deg),
                     center[2] + math.radians(yaw_span_deg) + math.radians(yaw_step_deg) / 2,
                     math.radians(yaw_step_deg))
    return [(np.array([x, y, yaw]), (ix, iy, iyaw))
            for iyaw, yaw in enumerate(yaws) for iy, y in enumerate(ys) for ix, x in enumerate(xs)
            if 0.03 <= x <= FIELD_LENGTH_M - .03 and 0.03 <= y <= FIELD_WIDTH_M - .03], xs, ys, yaws


def draw_result(image, blue, upper, lower, pose, lower_points, upper_points, height_m, pitch_deg):
    result = image.copy()
    result[blue > 0] = (190, 0, 190)
    result[upper > 0] = (0, 255, 0)
    result[lower > 0] = (0, 165, 255)
    for points, color in ((upper_points, (0, 255, 0)), (lower_points, (0, 0, 255))):
        projected, visible = project_visible(points, pose, height_m, pitch_deg, image.shape[:2])
        pixels = np.rint(projected[visible]).astype(np.int32)
        for point in pixels:
            cv2.circle(result, tuple(point), 1, color, -1, cv2.LINE_AA)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-map-match.jpg"))
    parser.add_argument("--center-x", type=float, default=1.5)
    parser.add_argument("--center-y", type=float, default=1.0)
    parser.add_argument("--center-yaw-deg", type=float, default=0.0)
    parser.add_argument("--span-m", type=float, default=1.45)
    parser.add_argument("--yaw-span-deg", type=float, default=180.0)
    parser.add_argument("--step-m", type=float, default=.15)
    parser.add_argument("--yaw-step-deg", type=float, default=12.0)
    parser.add_argument("--camera-height-m", type=float, default=.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    parser.add_argument("--samples-per-wall", type=int, default=48)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("cannot read {}".format(args.image))
    if image.shape[:2] != (720, 1280):
        parser.error("expected 1280x720 rectified image")
    thresholds = load_thresholds(args.thresholds)
    blue, upper, lower, upper_dt, lower_dt = observed_distance_fields(image, thresholds["blue_fence"])
    lower_points, upper_points = wall_points(args.samples_per_wall)
    candidates, xs, ys, yaws = candidate_grid(
        (args.center_x, args.center_y, math.radians(args.center_yaw_deg)), args.span_m,
        args.yaw_span_deg, args.step_m, args.yaw_step_deg)
    scores = np.full((len(yaws), len(ys), len(xs)), np.nan)
    for pose, (ix, iy, iyaw) in candidates:
        scores[iyaw, iy, ix] = score_pose(pose, lower_points, upper_points, lower_dt, upper_dt,
                                           args.camera_height_m, args.pitch_down_deg)
    best_index = np.unravel_index(np.nanargmin(scores), scores.shape)
    best_pose = np.array([xs[best_index[2]], ys[best_index[1]], yaws[best_index[0]]])
    best_score = scores[best_index]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), draw_result(image, blue, upper, lower, best_pose,
                                               lower_points, upper_points,
                                               args.camera_height_m, args.pitch_down_deg))
    print("candidates={} best_score_px={:.3f}".format(len(candidates), best_score))
    print("best_pose_m_deg: x={:.3f} y={:.3f} yaw={:.2f}".format(
        best_pose[0], best_pose[1], math.degrees(best_pose[2])))
    print("wrote {}".format(args.output))


if __name__ == "__main__":
    main()
