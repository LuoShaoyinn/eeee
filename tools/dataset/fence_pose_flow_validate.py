#!/usr/bin/env python3
"""Compare fence-pose-induced image motion against dense optical flow offline."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import (FENCE_HEIGHT_M, FIELD_LENGTH_M, FIELD_WIDTH_M,
                                 extract_corner_pairs, project_points, refine_joint_pose,
                                 refine_pose)
from fit_fence_edges import column_edges
from hsv_thresholds import load_thresholds


HEIGHT_M, PITCH_DEG = .1311825723, 30.1132498
OBJECT_CORNERS = np.array([[FIELD_LENGTH_M, FIELD_WIDTH_M, 0.0],
                           [FIELD_LENGTH_M, FIELD_WIDTH_M, FENCE_HEIGHT_M],
                           [FIELD_LENGTH_M, 0.0, 0.0],
                           [FIELD_LENGTH_M, 0.0, FENCE_HEIGHT_M]], dtype=np.float64)
WALL_SAMPLES = np.linspace([FIELD_LENGTH_M, 0.0], [FIELD_LENGTH_M, FIELD_WIDTH_M], 48)
WALL_POINTS = np.vstack((np.column_stack((WALL_SAMPLES, np.zeros(len(WALL_SAMPLES)))),
                         np.column_stack((WALL_SAMPLES, np.full(len(WALL_SAMPLES), FENCE_HEIGHT_M)))))


def estimate_pose(frame, blue_ranges, prior):
    blue, upper, lower, upper_lines, lower_lines = extract_corner_pairs(frame, blue_ranges, 3)
    _, _, _, upper_edge, lower_edge = column_edges(blue, min_pixels=8)
    observed = np.array([lower[0], upper[0], lower[1], upper[1]], dtype=np.float64)
    # A production EKF supplies this prior.  The first-frame seed is only for
    # this local offline test; global recovery belongs to the particle filter.
    seed = prior if prior is not None else np.array([.55, 1.0, 0.0])
    pose, corner_rms = refine_pose(OBJECT_CORNERS, observed, seed, HEIGHT_M, PITCH_DEG)
    pose, joint_corner_rms, line_rms = refine_joint_pose(
        pose, OBJECT_CORNERS, observed, upper_lines[1], lower_lines[1], .2, HEIGHT_M, PITCH_DEG)
    if joint_corner_rms > 16.0 or line_rms > 16.0:
        return None, (joint_corner_rms, line_rms)
    return pose, (joint_corner_rms, line_rms)


def flow_consistency(previous_pose, pose, flow):
    previous = project_points(WALL_POINTS, previous_pose, HEIGHT_M, PITCH_DEG)
    current = project_points(WALL_POINTS, pose, HEIGHT_M, PITCH_DEG)
    x, y = np.rint(previous).astype(int).T
    inside = ((x >= 0) & (x < flow.shape[1]) & (y >= 0) & (y < flow.shape[0]))
    measured = flow[y[inside], x[inside]]
    predicted = current[inside] - previous[inside]
    magnitude = np.linalg.norm(measured, axis=1)
    valid = (magnitude > .03) & (magnitude < 20.0)
    if valid.sum() < 12:
        return None
    measured, predicted = measured[valid], predicted[valid]
    error = np.linalg.norm(measured - predicted, axis=1)
    cosine = np.sum(measured * predicted, axis=1) / (
        np.linalg.norm(measured, axis=1) * np.linalg.norm(predicted, axis=1) + 1e-6)
    return float(np.median(error)), float(np.median(cosine)), int(valid.sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--frames", type=int, default=180)
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error("cannot open {}".format(args.video))
    thresholds = load_thresholds(args.thresholds)
    previous_frame = previous_pose = None
    measurements, valid_poses = [], 0
    for _ in range(args.frames):
        ok, frame = capture.read()
        if not ok:
            break
        try:
            pose, quality = estimate_pose(frame, thresholds["blue_fence"], previous_pose)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            pose = None
        if previous_frame is not None and previous_pose is not None and pose is not None:
            flow = cv2.calcOpticalFlowFarneback(cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY),
                                                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), None,
                                                .5, 3, 21, 3, 5, 1.2, 0)
            result = flow_consistency(previous_pose, pose, flow)
            if result is not None:
                measurements.append(result)
        if pose is not None:
            previous_pose, valid_poses = pose, valid_poses + 1
        else:
            previous_pose = None
        previous_frame = frame
    capture.release()
    if not measurements:
        raise SystemExit("no consecutive valid pose/flow comparisons")
    error, cosine, samples = np.median(measurements, axis=0)
    print("valid_pose_frames={}".format(valid_poses))
    print("pose_flow_pairs={} sampled_points_per_pair={}".format(len(measurements), round(samples)))
    print("median_flow_prediction_error_px={:.3f}".format(error))
    print("median_flow_direction_cosine={:.3f}".format(cosine))


if __name__ == "__main__":
    main()
