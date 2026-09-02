#!/usr/bin/env python3
"""Validate all-wall distance-field pose changes against dense optical flow."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from fence_map_likelihood import (candidate_grid, observed_distance_fields, project_visible,
                                  score_pose, wall_points)
from estimate_fence_pose import FIELD_LENGTH_M, FIELD_WIDTH_M
from hsv_thresholds import load_thresholds


HEIGHT_M, PITCH_DEG = .1311825723, 30.1132498


def draw_arena_trajectory(poses: np.ndarray, output: Path):
    """Draw the field-coordinate trajectory; +X is right and +Y is upward."""
    scale, margin = 260, 54
    width = round(FIELD_LENGTH_M * scale) + 2 * margin
    height = round(FIELD_WIDTH_M * scale) + 2 * margin
    image = np.full((height, width, 3), (238, 238, 238), np.uint8)

    def pixel(x, y):
        return (round(margin + x * scale), round(height - margin - y * scale))

    cv2.rectangle(image, pixel(0, FIELD_WIDTH_M), pixel(FIELD_LENGTH_M, 0), (200, 80, 30), 5)
    # Home and opponent home are 0.2 m by 0.3 m at diagonally opposite corners.
    cv2.rectangle(image, pixel(0, .3), pixel(.2, 0), (35, 35, 35), -1)
    cv2.rectangle(image, pixel(2.8, FIELD_WIDTH_M), pixel(3.0, FIELD_WIDTH_M - .3), (35, 35, 35), -1)
    cv2.putText(image, "A (0, 0)", pixel(.02, .06), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(image, "B (3, 0)", pixel(2.55, .06), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 0), 1, cv2.LINE_AA)

    points = np.asarray([pixel(x, y) for x, y, _ in poses], np.int32)
    if len(points) > 1:
        cv2.polylines(image, [points], False, (0, 115, 255), 2, cv2.LINE_AA)
    for index, ((x, y, yaw), point) in enumerate(zip(poses, points)):
        if index in (0, len(poses) - 1) or index % 5 == 0:
            end = pixel(x + .11 * math.cos(yaw), y + .11 * math.sin(yaw))
            cv2.arrowedLine(image, tuple(point), end, (0, 0, 210), 1, cv2.LINE_AA, 0, .25)
    if len(points):
        cv2.circle(image, tuple(points[0]), 6, (0, 170, 0), -1, cv2.LINE_AA)
        cv2.circle(image, tuple(points[-1]), 6, (0, 0, 220), -1, cv2.LINE_AA)
    cv2.putText(image, "green=start, red=end, arrows=camera forward", (margin, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image)


def find_best(candidates, lower_points, upper_points, lower_dt, upper_dt):
    best_score, best_pose = math.inf, None
    for pose, _ in candidates:
        score = score_pose(pose, lower_points, upper_points, lower_dt, upper_dt, HEIGHT_M, PITCH_DEG)
        if score < best_score:
            best_score, best_pose = score, pose
    return best_pose, best_score


def flow_agreement(previous_pose, pose, flow, lower_points, upper_points, previous_lower_dt,
                   previous_upper_dt):
    measured_parts, predicted_parts = [], []
    for points, prior_dt in ((lower_points, previous_lower_dt), (upper_points, previous_upper_dt)):
        previous, valid = project_visible(points, previous_pose, HEIGHT_M, PITCH_DEG, flow.shape[:2])
        current, _ = project_visible(points, pose, HEIGHT_M, PITCH_DEG, flow.shape[:2])
        x = np.rint(previous[:, 0]).astype(int)
        y = np.rint(previous[:, 1]).astype(int)
        inside = valid & (x >= 0) & (x < flow.shape[1]) & (y >= 0) & (y < flow.shape[0])
        x, y = x[inside], y[inside]
        # Score only projections that actually landed on the previous frame's edge.
        edge = prior_dt[y, x] < 3.0
        if np.count_nonzero(edge) < 8:
            continue
        measured_parts.append(flow[y[edge], x[edge]])
        predicted_parts.append(current[inside][edge] - previous[inside][edge])
    if not measured_parts:
        return None
    measured = np.vstack(measured_parts)
    predicted = np.vstack(predicted_parts)
    magnitude = np.linalg.norm(measured, axis=1)
    valid = (magnitude > .05) & (magnitude < 20.0)
    if np.count_nonzero(valid) < 12:
        return None
    measured, predicted = measured[valid], predicted[valid]
    error = np.linalg.norm(measured - predicted, axis=1)
    cosine = np.sum(measured * predicted, axis=1) / (
        np.linalg.norm(measured, axis=1) * np.linalg.norm(predicted, axis=1) + 1e-6)
    return float(np.median(error)), float(np.median(cosine)), int(np.count_nonzero(valid))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--trajectory-output", type=Path,
                        default=Path("/tmp/fence-map-trajectory.jpg"))
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error("cannot open {}".format(args.video))
    thresholds = load_thresholds(args.thresholds)
    lower_points, upper_points = wall_points(48)
    previous_frame = previous_pose = previous_lower_dt = previous_upper_dt = None
    agreements, poses, scores = [], [], []
    frame_index = 0
    while frame_index < args.frames:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % args.stride:
            frame_index += 1
            continue
        _, _, _, upper_dt, lower_dt = observed_distance_fields(frame, thresholds["blue_fence"])
        if previous_pose is None:
            candidates, _, _, _ = candidate_grid((1.5, 1.0, 0.0), 1.45, 180.0, .15, 12.0)
        else:
            candidates, _, _, _ = candidate_grid(previous_pose, .30, 15.0, .10, 5.0)
        pose, score = find_best(candidates, lower_points, upper_points, lower_dt, upper_dt)
        if pose is not None:
            if previous_frame is not None and previous_pose is not None:
                flow = cv2.calcOpticalFlowFarneback(
                    cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY),
                    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), None, .5, 3, 21, 3, 5, 1.2, 0)
                result = flow_agreement(previous_pose, pose, flow, lower_points, upper_points,
                                        previous_lower_dt, previous_upper_dt)
                if result is not None:
                    agreements.append(result)
            poses.append(pose)
            scores.append(score)
            previous_pose = pose
        previous_frame, previous_lower_dt, previous_upper_dt = frame, lower_dt, upper_dt
        frame_index += 1
    capture.release()
    if not poses:
        raise SystemExit("no pose estimates")
    poses = np.asarray(poses)
    draw_arena_trajectory(poses, args.trajectory_output)
    print("localized_frames={} median_score_px={:.3f}".format(len(poses), np.median(scores)))
    print("first_pose: x={:.3f} y={:.3f} yaw_deg={:.2f}".format(
        poses[0, 0], poses[0, 1], math.degrees(poses[0, 2])))
    print("last_pose:  x={:.3f} y={:.3f} yaw_deg={:.2f}".format(
        poses[-1, 0], poses[-1, 1], math.degrees(poses[-1, 2])))
    print("wrote {}".format(args.trajectory_output))
    if agreements:
        error, cosine, support = np.median(agreements, axis=0)
        print("flow_pairs={} median_edge_support={}".format(len(agreements), round(support)))
        print("median_flow_prediction_error_px={:.3f}".format(error))
        print("median_flow_direction_cosine={:.3f}".format(cosine))
    else:
        print("flow_pairs=0 (too little measurable fence motion in this sequence)")


if __name__ == "__main__":
    main()
