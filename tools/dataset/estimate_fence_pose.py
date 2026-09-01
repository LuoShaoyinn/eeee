#!/usr/bin/env python3
"""Estimate camera field pose from paired top/bottom edges of the far blue fence."""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from fit_fence_edges import (column_edges, hough_panel_lines,
                             retain_fence_components)
from hsv_thresholds import load_thresholds, make_mask


FIELD_LENGTH_M = 3.0
FIELD_WIDTH_M = 1.985
FENCE_HEIGHT_M = 0.254
CAMERA_MATRIX = np.array([[400.0, 0.0, 640.0],
                          [0.0, 400.0, 360.0],
                          [0.0, 0.0, 1.0]], dtype=np.float64)


def line_intersection(first: np.ndarray, second: np.ndarray) -> np.ndarray | None:
    denominator = first[0] - second[0]
    if abs(denominator) < 1e-6:
        return None
    x = (second[1] - first[1]) / denominator
    return np.array([x, np.polyval(first, x)], dtype=np.float64)


def extract_corner_pairs(image: np.ndarray, blue_ranges, maximum: int):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = make_mask(hsv, blue_ranges)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    blue = retain_fence_components(blue)
    _, _, _, upper_edge, lower_edge = column_edges(blue, min_pixels=8)
    upper_lines = hough_panel_lines(upper_edge, maximum)
    lower_lines = hough_panel_lines(lower_edge, maximum)
    if len(upper_lines) != len(lower_lines) or len(upper_lines) < 3:
        raise ValueError("need three matching top and bottom fence panels")
    upper = [line_intersection(upper_lines[index], upper_lines[index + 1])
             for index in range(len(upper_lines) - 1)]
    lower = [line_intersection(lower_lines[index], lower_lines[index + 1])
             for index in range(len(lower_lines) - 1)]
    if any(point is None for point in [*upper, *lower]):
        raise ValueError("adjacent fitted fence lines are parallel")
    return blue, np.asarray(upper), np.asarray(lower), upper_lines, lower_lines


def pose_to_extrinsics(pose: np.ndarray, camera_height_m: float, pitch_down_deg: float):
    """Build OpenCV world-to-camera extrinsics for a level fixed camera."""
    _, _, yaw = pose
    pitch = math.radians(pitch_down_deg)
    sine, cosine = math.sin(pitch), math.cos(pitch)
    # Camera axes are OpenCV right/down/forward. Car axes are forward/left/up.
    rotation_car_from_camera = np.array([
        [0.0, -sine, cosine],
        [-1.0, 0.0, 0.0],
        [0.0, -cosine, -sine],
    ], dtype=np.float64)
    rotation_field_from_car = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    rotation_world_to_camera = (rotation_field_from_car @ rotation_car_from_camera).T
    camera_field = np.array([pose[0], pose[1], camera_height_m], dtype=np.float64)
    translation = -rotation_world_to_camera @ camera_field
    rotation_vector, _ = cv2.Rodrigues(rotation_world_to_camera)
    return rotation_vector, translation.reshape(3, 1)


def project_points(points: np.ndarray, pose: np.ndarray, camera_height_m: float,
                   pitch_down_deg: float) -> np.ndarray:
    rotation, translation = pose_to_extrinsics(pose, camera_height_m, pitch_down_deg)
    projected, _ = cv2.projectPoints(points, rotation, translation, CAMERA_MATRIX,
                                     np.zeros((4, 1)))
    return projected.reshape(-1, 2)


def project_fence_edges(pose: np.ndarray, camera_height_m: float, pitch_down_deg: float,
                        points_per_wall: int = 100):
    """Project the observed far x=3 m panel's lower and upper 3D edges."""
    xy = np.linspace([FIELD_LENGTH_M, 0.0], [FIELD_LENGTH_M, FIELD_WIDTH_M], points_per_wall)
    return (project_points(np.column_stack((xy, np.zeros(points_per_wall))),
                           pose, camera_height_m, pitch_down_deg),
            project_points(np.column_stack((xy, np.full(points_per_wall, FENCE_HEIGHT_M))),
                           pose, camera_height_m, pitch_down_deg))


def line_distance(points: np.ndarray, line: np.ndarray) -> np.ndarray:
    """Signed perpendicular pixel distance to y = slope*x + intercept."""
    slope, intercept = line
    return (points[:, 1] - slope * points[:, 0] - intercept) / math.hypot(slope, 1.0)


def joint_residual(pose: np.ndarray, object_points: np.ndarray, corner_points: np.ndarray,
                   upper_line: np.ndarray, lower_line: np.ndarray, line_weight: float,
                   camera_height_m: float, pitch_down_deg: float) -> np.ndarray:
    """Corner anchors plus soft upper/lower-panel alignment constraints."""
    corner_residual = (project_points(
        object_points, pose, camera_height_m, pitch_down_deg) - corner_points).reshape(-1)
    lower, upper = project_fence_edges(pose, camera_height_m, pitch_down_deg, points_per_wall=16)
    alignment = np.concatenate((line_distance(lower, lower_line), line_distance(upper, upper_line)))
    return np.concatenate((corner_residual, math.sqrt(line_weight) * alignment))


def refine_joint_pose(initial: np.ndarray, object_points: np.ndarray, corner_points: np.ndarray,
                      upper_line: np.ndarray, lower_line: np.ndarray, line_weight: float,
                      camera_height_m: float, pitch_down_deg: float):
    """Damped Gauss--Newton pose fit with corners dominant over line alignment."""
    pose = initial.astype(np.float64).copy()
    for _ in range(60):
        residual = joint_residual(pose, object_points, corner_points, upper_line, lower_line,
                                  line_weight, camera_height_m, pitch_down_deg)
        jacobian = np.empty((len(residual), 3), dtype=np.float64)
        for axis, step in enumerate((1e-4, 1e-4, 1e-4)):
            shifted = pose.copy()
            shifted[axis] += step
            shifted_residual = joint_residual(
                shifted, object_points, corner_points, upper_line, lower_line,
                line_weight, camera_height_m, pitch_down_deg)
            jacobian[:, axis] = (shifted_residual - residual) / step
        update = np.linalg.solve(jacobian.T @ jacobian + 0.01 * np.eye(3),
                                 -jacobian.T @ residual)
        pose += update
        pose[2] = (pose[2] + math.pi) % (2 * math.pi) - math.pi
        if np.linalg.norm(update) < 1e-7:
            break
    projected = project_points(object_points, pose, camera_height_m, pitch_down_deg)
    corner_rms = float(np.sqrt(np.mean(np.square(projected - corner_points))))
    lower, upper = project_fence_edges(pose, camera_height_m, pitch_down_deg, points_per_wall=16)
    line_rms = float(np.sqrt(np.mean(np.square(np.concatenate((
        line_distance(lower, lower_line), line_distance(upper, upper_line)))))))
    return pose, corner_rms, line_rms


def far_wall_metrics(pose: np.ndarray):
    """Return the perpendicular and camera-forward distances to the B-D wall."""
    wall_distance_m = FIELD_LENGTH_M - pose[0]
    forward_x = math.cos(pose[2])
    if forward_x <= 1e-6:
        raise ValueError("camera is not facing the x=3 m wall")
    forward_distance_m = wall_distance_m / forward_x
    hit_y_m = pose[1] + forward_distance_m * math.sin(pose[2])
    if not 0.0 <= hit_y_m <= FIELD_WIDTH_M:
        raise ValueError("camera forward ray misses the B-D wall segment")
    return wall_distance_m, forward_distance_m, hit_y_m


def nearest_samples(image: np.ndarray, points: np.ndarray):
    x = np.rint(points[:, 0]).astype(int)
    y = np.rint(points[:, 1]).astype(int)
    valid = (x >= 0) & (x < image.shape[1]) & (y >= 0) & (y < image.shape[0])
    values = np.zeros(len(points), dtype=image.dtype)
    values[valid] = image[y[valid], x[valid]]
    return values, valid


def dark_vertical_seam(top: np.ndarray, bottom: np.ndarray, value: np.ndarray,
                       blue: np.ndarray, maximum_shift_px: int = 16):
    """Find a dark, vertically continuous seam with blue material on both sides."""
    direction = bottom - top
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    candidates = []
    for shift in range(-maximum_shift_px, maximum_shift_px + 1):
        fraction = np.linspace(.14, .86, 32)
        centre = top[None, :] * (1 - fraction[:, None]) + bottom[None, :] * fraction[:, None]
        centre += np.array([shift, 0.0])
        left, right = centre + normal * 4.0, centre - normal * 4.0
        centre_value, centre_valid = nearest_samples(value, centre)
        left_value, left_valid = nearest_samples(value, left)
        right_value, right_valid = nearest_samples(value, right)
        centre_blue, _ = nearest_samples(blue, centre)
        left_blue, _ = nearest_samples(blue, left)
        right_blue, _ = nearest_samples(blue, right)
        valid = (centre_valid & left_valid & right_valid & (centre_blue > 0) &
                 (left_blue > 0) & (right_blue > 0))
        contrast = np.maximum(0.0, (left_value.astype(float) + right_value.astype(float)) / 2 -
                              centre_value.astype(float))[valid]
        score = float(np.mean(np.sort(contrast)[-max(4, len(contrast) // 2):])) if len(contrast) else 0.0
        candidates.append((score, shift))
    candidates.sort(reverse=True)
    score, shift = candidates[0]
    margin = score - candidates[1][0] if len(candidates) > 1 else 0.0
    return {
        "score": score,
        "shift": shift,
        "margin": margin,
        "accepted": score >= 6.0 and margin >= 0.4 and abs(shift) < maximum_shift_px,
        "top": top + np.array([shift, 0.0]),
        "bottom": bottom + np.array([shift, 0.0]),
    }


def refine_pose(object_points: np.ndarray, image_points: np.ndarray, initial: np.ndarray,
                camera_height_m: float, pitch_down_deg: float):
    """Damped Gauss--Newton over the only free state: field x, y, and yaw."""
    pose = initial.astype(np.float64).copy()
    for _ in range(60):
        residual = (project_points(object_points, pose, camera_height_m, pitch_down_deg) -
                    image_points).reshape(-1)
        jacobian = np.empty((len(residual), 3), dtype=np.float64)
        for axis, step in enumerate((1e-4, 1e-4, 1e-4)):
            shifted = pose.copy()
            shifted[axis] += step
            shifted_residual = (project_points(
                object_points, shifted, camera_height_m, pitch_down_deg) - image_points).reshape(-1)
            jacobian[:, axis] = (shifted_residual - residual) / step
        update = np.linalg.solve(jacobian.T @ jacobian + 0.01 * np.eye(3),
                                 -jacobian.T @ residual)
        pose += update
        pose[2] = (pose[2] + math.pi) % (2 * math.pi) - math.pi
        if np.linalg.norm(update) < 1e-7:
            break
    rms_px = float(np.sqrt(np.mean(np.square(
        project_points(object_points, pose, camera_height_m, pitch_down_deg) - image_points))))
    return pose, rms_px


def solve_far_wall(upper: np.ndarray, lower: np.ndarray, camera_height_m: float,
                   pitch_down_deg: float):
    """Solve B/D from two vertical pairs on the far x=3 m wall.

    In this forward-facing view, D (y=max) appears at the left image join and
    B (y=0) appears at the right join.  Each top/lower image pair is one 3D
    vertical fence corner, which is the geometric constraint missing from a
    separate 2D line fit.
    """
    object_points = np.array([
        [FIELD_LENGTH_M, FIELD_WIDTH_M, 0.0],
        [FIELD_LENGTH_M, FIELD_WIDTH_M, FENCE_HEIGHT_M],
        [FIELD_LENGTH_M, 0.0, 0.0],
        [FIELD_LENGTH_M, 0.0, FENCE_HEIGHT_M],
    ], dtype=np.float64)
    image_points = np.array([lower[0], upper[0], lower[1], upper[1]], dtype=np.float64)
    # The far x=3 wall is visible in the middle of these forward-facing frames.
    # A small grid makes this robust to an unknown initial location; the live
    # tracker will instead start from its prior wheel/IMU pose.
    candidates = []
    for x in np.linspace(0.15, 2.6, 5):
        for y in np.linspace(0.15, FIELD_WIDTH_M - 0.15, 5):
            for yaw in np.linspace(-0.7, 0.7, 7):
                pose, rms_px = refine_pose(
                    object_points, image_points, np.array([x, y, yaw]),
                    camera_height_m, pitch_down_deg)
                if 0.0 < pose[0] < FIELD_LENGTH_M and 0.0 < pose[1] < FIELD_WIDTH_M:
                    candidates.append((rms_px, pose))
    if not candidates:
        raise ValueError("no physically inside-field pose")
    rms_px, pose = min(candidates, key=lambda candidate: candidate[0])
    rotation_vector, translation = pose_to_extrinsics(pose, camera_height_m, pitch_down_deg)
    camera_field = np.array([pose[0], pose[1], camera_height_m], dtype=np.float64)
    return rotation_vector, translation, camera_field, math.degrees(pose[2]), rms_px, image_points


def draw_pose(image, rotation_vector, translation, observed, lower_edge, upper_edge, dark_seams):
    result = image.copy()
    result[upper_edge > 0] = (0, 120, 0)
    result[lower_edge > 0] = (0, 80, 120)
    # The detected corner pairs are cyan. The rigid 3D-wall reprojection is red.
    for point in observed:
        cv2.circle(result, tuple(np.rint(point).astype(int)), 5, (255, 255, 0), -1, cv2.LINE_AA)
    corners = np.array([[FIELD_LENGTH_M, 0.0], [FIELD_LENGTH_M, FIELD_WIDTH_M]], dtype=np.float64)
    lower_vertices = np.column_stack((corners, np.zeros(2)))
    upper_vertices = np.column_stack((corners, np.full(2, FENCE_HEIGHT_M)))
    lower_projected, _ = cv2.projectPoints(lower_vertices, rotation_vector, translation,
                                            CAMERA_MATRIX, np.zeros((4, 1)))
    upper_projected, _ = cv2.projectPoints(upper_vertices, rotation_vector, translation,
                                            CAMERA_MATRIX, np.zeros((4, 1)))
    lower_points = np.rint(lower_projected.reshape(-1, 2)).astype(int)
    upper_points = np.rint(upper_projected.reshape(-1, 2)).astype(int)
    cv2.line(result, tuple(lower_points[0]), tuple(lower_points[1]), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.line(result, tuple(upper_points[0]), tuple(upper_points[1]), (0, 255, 0), 2, cv2.LINE_AA)
    for lower_point, upper_point in zip(lower_points, upper_points):
        cv2.line(result, tuple(np.rint(lower_point).astype(int)),
                 tuple(np.rint(upper_point).astype(int)), (255, 0, 0), 1, cv2.LINE_AA)
    for label, point in zip(("B", "D"), lower_points):
        cv2.putText(result, label, tuple(point), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(result, label, tuple(point), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (0, 0, 0), 1, cv2.LINE_AA)
    for label, seam in dark_seams.items():
        color = (255, 0, 255) if seam["accepted"] else (0, 255, 255)
        cv2.line(result, tuple(np.rint(seam["top"]).astype(int)),
                 tuple(np.rint(seam["bottom"]).astype(int)), color, 2, cv2.LINE_AA)
        cv2.putText(result, "{} dark".format(label), tuple(np.rint(seam["top"]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, help="write the observed/reprojected wall overlay")
    parser.add_argument("--camera-height-m", type=float, default=0.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    parser.add_argument("--line-alignment-weight", type=float, default=0.2,
                        help="soft line-fit weight relative to cyan corner anchors")
    args = parser.parse_args()
    try:
        image = cv2.imread(str(args.image))
        if image is None:
            raise ValueError("cannot read {}".format(args.image))
        if image.shape[:2] != (720, 1280):
            raise ValueError("expected a 1280x720 rectified image")
        thresholds = load_thresholds(args.thresholds)
        blue, upper, lower, upper_lines, lower_lines = extract_corner_pairs(
            image, thresholds["blue_fence"], maximum=3)
        _, _, _, upper_edge, lower_edge = column_edges(blue, min_pixels=8)
        if args.camera_height_m <= 0 or args.line_alignment_weight < 0:
            parser.error("camera height must be positive and line alignment weight non-negative")
        rotation, translation, camera_field, yaw_deg, initial_rms_px, observed = solve_far_wall(
            upper, lower, args.camera_height_m, args.pitch_down_deg)
        initial_pose = np.array([camera_field[0], camera_field[1], math.radians(yaw_deg)])
        object_points = np.array([
            [FIELD_LENGTH_M, FIELD_WIDTH_M, 0.0], [FIELD_LENGTH_M, FIELD_WIDTH_M, FENCE_HEIGHT_M],
            [FIELD_LENGTH_M, 0.0, 0.0], [FIELD_LENGTH_M, 0.0, FENCE_HEIGHT_M],
        ], dtype=np.float64)
        refined_pose, corner_error_px, line_error_px = refine_joint_pose(
            initial_pose, object_points, observed, upper_lines[1], lower_lines[1],
            args.line_alignment_weight, args.camera_height_m, args.pitch_down_deg)
        rotation, translation = pose_to_extrinsics(
            refined_pose, args.camera_height_m, args.pitch_down_deg)
        camera_field = np.array([refined_pose[0], refined_pose[1], args.camera_height_m])
        yaw_deg = math.degrees(refined_pose[2])
        wall_distance_m, forward_distance_m, forward_hit_y_m = far_wall_metrics(refined_pose)
        corner_3d = {
            "B": np.array([[FIELD_LENGTH_M, 0.0, FENCE_HEIGHT_M],
                           [FIELD_LENGTH_M, 0.0, 0.0]]),
            "D": np.array([[FIELD_LENGTH_M, FIELD_WIDTH_M, FENCE_HEIGHT_M],
                           [FIELD_LENGTH_M, FIELD_WIDTH_M, 0.0]]),
        }
        value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2]
        dark_seams = {
            name: dark_vertical_seam(*project_points(
                points, refined_pose, args.camera_height_m, args.pitch_down_deg), value, blue)
            for name, points in corner_3d.items()
        }
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.output), draw_pose(
                    image, rotation, translation, observed, lower_edge, upper_edge, dark_seams)):
                raise ValueError("cannot write {}".format(args.output))
    except (OSError, ValueError, cv2.error) as error:
        print("pose estimate failed: {}".format(error), file=sys.stderr)
        return 1
    print("camera_field_m: x={:.3f} y={:.3f} z={:.3f}".format(*camera_field))
    print("camera_yaw_deg: {:.2f}".format(yaw_deg))
    print("initial_corner_rms_px: {:.3f}".format(initial_rms_px))
    print("joint_corner_rms_px: {:.3f}".format(corner_error_px))
    print("joint_line_rms_px: {:.3f}".format(line_error_px))
    print("B-D wall distance_m: perpendicular={:.3f} forward={:.3f} hit_y={:.3f}".format(
        wall_distance_m, forward_distance_m, forward_hit_y_m))
    for name in ("B", "D"):
        seam = dark_seams[name]
        print("dark_seam_{}: shift_px={:+d} score={:.3f} margin={:.3f} accepted={}".format(
            name, seam["shift"], seam["score"], seam["margin"], seam["accepted"]))
    if args.output:
        print("wrote {}".format(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
