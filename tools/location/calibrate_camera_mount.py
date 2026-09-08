#!/usr/bin/env python3
"""Estimate a fixed camera's height and pitch from a level floor checkerboard."""

import argparse
import glob
import math
import sys
from pathlib import Path

import cv2
import numpy as np


def read_rectified_intrinsics(path):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open camera calibration: {}".format(path))
    try:
        camera_matrix = storage.getNode("rectified_K").mat()
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
    finally:
        storage.release()
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise ValueError("camera calibration must contain rectified_K")
    return camera_matrix.astype(np.float64), width, height


def make_object_points(columns, rows, square_m, row_offset):
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:columns, row_offset:row_offset + rows].T.reshape(-1, 2)
    points *= square_m
    return points


def find_checkerboard(gray, full_pattern_size, usable_rows, search_roi_top):
    offset = int(gray.shape[0] * search_roi_top)
    found, corners = cv2.findChessboardCorners(
        gray[offset:, :], full_pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        corners[:, :, 1] += offset
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
        columns, rows = full_pattern_size
        corners = corners.reshape(rows, columns, 1, 2)[-usable_rows:].reshape(-1, 1, 2)
    return found, corners


def solve_board_pose(image, object_points, full_pattern_size, usable_rows, camera_matrix,
                     search_roi_top):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = find_checkerboard(
        gray, full_pattern_size, usable_rows, search_roi_top)
    if not found:
        return None
    solved, rotation_vector, translation = cv2.solvePnP(
        object_points, corners, camera_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_ITERATIVE)
    if not solved:
        return None
    rotation_board_to_camera, _ = cv2.Rodrigues(rotation_vector)
    projected, _ = cv2.projectPoints(
        object_points, rotation_vector, translation, camera_matrix, np.zeros((4, 1)))
    errors = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    return rotation_board_to_camera, translation.reshape(3, 1), float(np.sqrt(np.mean(errors ** 2)))


def estimate_mount(rotation_board_to_camera, translation_board_to_camera, plane_height_m):
    """Return camera optical-centre height and forward-axis downward pitch."""
    rotation_camera_to_board = rotation_board_to_camera.T
    camera_position_board = -rotation_camera_to_board @ translation_board_to_camera

    # Chessboard Z has an arbitrary sign for a planar pose.  The normal from
    # the board to the camera is the physical upward normal for a floor board.
    normal_up = np.array([0.0, 0.0, 1.0])
    if camera_position_board[2, 0] < 0:
        normal_up *= -1
    height_m = plane_height_m + abs(float(camera_position_board[2, 0]))

    camera_forward_board = rotation_camera_to_board @ np.array([[0.0], [0.0], [1.0]])
    vertical_component = float(normal_up @ camera_forward_board[:, 0])
    horizontal_component = math.sqrt(max(0.0, 1.0 - vertical_component ** 2))
    pitch_down_deg = math.degrees(math.atan2(-vertical_component, horizontal_component))
    return height_m, pitch_down_deg, camera_position_board


def write_result(path, camera_matrix, width, height, measurements, plane_height_m):
    heights = np.array([measurement["height_m"] for measurement in measurements])
    pitches = np.array([measurement["pitch_down_deg"] for measurement in measurements])
    reprojection = np.array([measurement["reprojection_px"] for measurement in measurements])
    path.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise ValueError("cannot write {}".format(path))
    try:
        storage.write("model", "rectilinear_after_fisheye_undistortion")
        storage.write("image_width", width)
        storage.write("image_height", height)
        storage.write("K", camera_matrix)
        storage.write("D", np.zeros((4, 1), dtype=np.float64))
        storage.startWriteStruct("camera_mount", cv2.FileNode_MAP)
        storage.write("reference", "level checkerboard plane")
        storage.write("checkerboard_plane_height_above_ground_m", plane_height_m)
        storage.write("camera_optical_center_height_m", float(np.mean(heights)))
        storage.write("camera_optical_center_height_std_m", float(np.std(heights, ddof=0)))
        storage.write("optical_axis_pitch_down_deg", float(np.mean(pitches)))
        storage.write("optical_axis_pitch_std_deg", float(np.std(pitches, ddof=0)))
        storage.write("pitch_convention", "positive means camera optical axis points down from horizontal")
        storage.write("checkerboard_reprojection_rms_px",
                      float(math.sqrt(np.mean(np.square(reprojection)))))
        storage.write("sample_count", len(measurements))
        storage.endWriteStruct()
    finally:
        storage.release()
    return float(np.mean(heights)), float(np.mean(pitches))


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate a fixed camera's height and pitch using a horizontal checkerboard.")
    parser.add_argument("--images", type=Path, nargs="+", required=True,
                        help="rectified 1280x720 checkerboard images; shell globs are supported")
    parser.add_argument("--pattern-cols", type=int, required=True,
                        help="checkerboard inner-corner count along width")
    parser.add_argument("--pattern-rows", type=int, required=True,
                        help="checkerboard inner-corner count along height")
    parser.add_argument("--usable-rows", type=int,
                        help="number of bottom inner-corner rows to use (default: all)")
    parser.add_argument("--row-offset", type=int,
                        help="first physical checkerboard row used (default: bottom rows)")
    parser.add_argument("--search-roi-top", type=float, default=0.0,
                        help="ignore this fraction of image height at the top")
    parser.add_argument("--square-m", type=float, required=True,
                        help="checkerboard square size in metres")
    parser.add_argument("--plane-height-m", type=float, default=0.0,
                        help="checkerboard printed-plane height above ground in metres")
    parser.add_argument("--max-reprojection-px", type=float, default=0.8)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--camera-calibration", type=Path,
                        default=Path(__file__).resolve().parents[2] / "config" / "calibration" /
                        "camera1_fisheye_1280x720_rectilinear_f400.yaml")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[2] / "config" / "calibration" /
                        "camera1_mount.yaml")
    args = parser.parse_args()
    usable_rows = args.usable_rows or args.pattern_rows
    row_offset = args.row_offset
    if row_offset is None:
        row_offset = args.pattern_rows - usable_rows
    if (args.pattern_cols < 3 or args.pattern_rows < 3 or args.square_m <= 0 or
            usable_rows < 3 or usable_rows > args.pattern_rows or row_offset < 0 or
            row_offset + usable_rows > args.pattern_rows):
        parser.error("pattern dimensions must be at least 3 and square-m must be positive")
    if (args.plane_height_m < 0 or args.max_reprojection_px <= 0 or args.min_samples < 1 or
            not 0 <= args.search_roi_top < 1):
        parser.error("plane height, reprojection limit, and min-samples must be valid")

    try:
        camera_matrix, width, height = read_rectified_intrinsics(args.camera_calibration)
        object_points = make_object_points(
            args.pattern_cols, usable_rows, args.square_m, row_offset)
        image_paths = []
        for requested_path in args.images:
            matches = [Path(match) for match in sorted(glob.glob(str(requested_path)))]
            image_paths.extend(matches or [requested_path])
        measurements = []
        for image_path in image_paths:
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                print("skip {}: cannot read image".format(image_path), file=sys.stderr)
                continue
            if image.shape[:2] != (height, width):
                print("skip {}: expected {}x{} rectified image".format(
                    image_path, width, height), file=sys.stderr)
                continue
            pose = solve_board_pose(
                image, object_points, (args.pattern_cols, args.pattern_rows), usable_rows,
                camera_matrix, args.search_roi_top)
            if pose is None:
                print("skip {}: checkerboard not found".format(image_path), file=sys.stderr)
                continue
            rotation, translation, error_px = pose
            if error_px > args.max_reprojection_px:
                print("skip {}: reprojection error {:.3f}px exceeds {:.3f}px".format(
                    image_path, error_px, args.max_reprojection_px), file=sys.stderr)
                continue
            mount = estimate_mount(rotation, translation, args.plane_height_m)
            measurements.append({
                "height_m": mount[0],
                "pitch_down_deg": mount[1],
                "reprojection_px": error_px,
            })
    except (OSError, ValueError, cv2.error) as error:
        print("calibration failed: {}".format(error), file=sys.stderr)
        return 1

    if len(measurements) < args.min_samples:
        print("calibration failed: only {} valid samples; need {}".format(
            len(measurements), args.min_samples), file=sys.stderr)
        return 1
    try:
        camera_height, pitch_down = write_result(
            args.output, camera_matrix, width, height, measurements, args.plane_height_m)
    except (OSError, ValueError, cv2.error) as error:
        print("calibration failed: {}".format(error), file=sys.stderr)
        return 1

    height_std = float(np.std([item["height_m"] for item in measurements]))
    pitch_std = float(np.std([item["pitch_down_deg"] for item in measurements]))
    print("wrote {}".format(args.output))
    print("camera optical-centre height: {:.4f} m (std {:.4f} m)".format(
        camera_height, height_std))
    print("optical-axis pitch: {:+.2f} deg, positive means downward (std {:.2f} deg)".format(
        pitch_down, pitch_std))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
