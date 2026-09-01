#!/usr/bin/env python3
"""Calibrate a rectified camera's rotation relative to an IMU from a chessboard."""

import argparse
import csv
import math
import sys
from pathlib import Path

import cv2
import numpy as np


METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


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
        raise ValueError("camera calibration must contain a 3x3 rectified_K matrix")
    if width <= 0 or height <= 0:
        raise ValueError("camera calibration must contain image_width and image_height")
    return camera_matrix.astype(np.float64), width, height


def required_float(row, names, row_number):
    for name in names:
        value = row.get(name)
        if value not in (None, ""):
            try:
                return float(value)
            except ValueError as error:
                raise ValueError("CSV row {} has invalid {}: {!r}".format(
                    row_number, name, value)) from error
    raise ValueError("CSV row {} is missing one of: {}".format(
        row_number, ", ".join(names)))


def quaternion_to_matrix(qw, qx, qy, qz):
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("IMU quaternion has zero length")
    qw, qx, qy, qz = quaternion / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def rotation_angle_deg(rotation):
    vector, _ = cv2.Rodrigues(rotation)
    return math.degrees(float(np.linalg.norm(vector)))


def average_rotations(rotations):
    accumulator = sum(rotations, np.zeros((3, 3), dtype=np.float64))
    left, _, right = np.linalg.svd(accumulator)
    result = left @ right
    if np.linalg.det(result) < 0:
        left[:, -1] *= -1
        result = left @ right
    return result


def chessboard_object_points(columns, rows, square_m):
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points *= square_m
    return points


def find_checkerboard(gray, pattern_size):
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    return found, corners


def find_board_pose(image, object_points, pattern_size, camera_matrix):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = find_checkerboard(gray, pattern_size)
    if not found:
        return None
    solved, rotation_vector, translation = cv2.solvePnP(
        object_points, corners, camera_matrix, np.zeros((4, 1)), flags=cv2.SOLVEPNP_ITERATIVE)
    if not solved:
        return None
    rotation, _ = cv2.Rodrigues(rotation_vector)
    projected, _ = cv2.projectPoints(
        object_points, rotation_vector, translation, camera_matrix, np.zeros((4, 1)))
    reprojection = np.linalg.norm(
        projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    return rotation, translation.reshape(3, 1), float(np.sqrt(np.mean(reprojection ** 2)))


def load_samples(csv_path, image_directory, object_points, pattern_size, camera_matrix,
                 expected_width, expected_height):
    accepted = []
    rejected = []
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("IMU CSV has no header")
        for row_number, row in enumerate(reader, start=2):
            image_name = row.get("image", "")
            if not image_name:
                rejected.append((row_number, "missing image"))
                continue
            try:
                rotation_imu_to_world = quaternion_to_matrix(
                    required_float(row, ("qw", "q_w"), row_number),
                    required_float(row, ("qx", "q_x"), row_number),
                    required_float(row, ("qy", "q_y"), row_number),
                    required_float(row, ("qz", "q_z"), row_number),
                )
            except ValueError as error:
                rejected.append((row_number, str(error)))
                continue
            image_path = Path(image_name)
            if not image_path.is_absolute():
                image_path = image_directory / image_path
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                rejected.append((row_number, "cannot read {}".format(image_path)))
                continue
            if image.shape[1] != expected_width or image.shape[0] != expected_height:
                rejected.append((row_number, "{} is {}x{}, expected {}x{}".format(
                    image_path, image.shape[1], image.shape[0], expected_width, expected_height)))
                continue
            board_pose = find_board_pose(image, object_points, pattern_size, camera_matrix)
            if board_pose is None:
                rejected.append((row_number, "checkerboard not found in {}".format(image_path)))
                continue
            rotation_board_to_camera, translation_board_to_camera, error_px = board_pose
            accepted.append({
                "row": row_number,
                "image": str(image_path),
                "rotation_imu_to_world": rotation_imu_to_world,
                "rotation_board_to_camera": rotation_board_to_camera,
                "translation_board_to_camera": translation_board_to_camera,
                "reprojection_px": error_px,
            })
    return accepted, rejected


def calibrate_rotation(samples, method):
    rotations_imu_to_world = [sample["rotation_imu_to_world"] for sample in samples]
    rotations_board_to_camera = [sample["rotation_board_to_camera"] for sample in samples]
    zero_translations = [np.zeros((3, 1), dtype=np.float64) for _ in samples]
    rotation_camera_to_imu, _ = cv2.calibrateHandEye(
        rotations_imu_to_world,
        zero_translations,
        rotations_board_to_camera,
        zero_translations,
        method=METHODS[method],
    )
    rotation_imu_to_camera = rotation_camera_to_imu.T

    # The board is static in the world. Estimate its fixed world orientation
    # from every accepted pose, then evaluate the orientation residual.
    board_to_world_estimates = [
        sample["rotation_imu_to_world"] @ rotation_camera_to_imu @
        sample["rotation_board_to_camera"]
        for sample in samples
    ]
    rotation_board_to_world = average_rotations(board_to_world_estimates)
    residuals = [
        rotation_angle_deg(
            (rotation_imu_to_camera @ sample["rotation_imu_to_world"].T @
             rotation_board_to_world) @ sample["rotation_board_to_camera"].T)
        for sample in samples
    ]
    relative_angles = [
        rotation_angle_deg(sample["rotation_imu_to_world"] @
                           samples[0]["rotation_imu_to_world"].T)
        for sample in samples[1:]
    ]
    return rotation_imu_to_camera, residuals, relative_angles


def write_result(path, camera_matrix, width, height, rotation_imu_to_camera, samples,
                 residuals, relative_angles, translation_imu_to_camera, measured_distance):
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
        storage.startWriteStruct("camera_imu", cv2.FileNode_MAP)
        storage.write("rotation_imu_to_camera", rotation_imu_to_camera)
        storage.write("rotation_camera_to_imu", rotation_imu_to_camera.T)
        storage.write("rotation_convention",
                      "p_camera = R_imu_to_camera * p_imu + t_imu_to_camera")
        if translation_imu_to_camera is None:
            storage.write("translation_status",
                          "unresolved: distance alone cannot determine a 3D translation vector")
            if measured_distance is not None:
                storage.write("imu_camera_center_distance_m", measured_distance)
        else:
            vector_imu = np.asarray(translation_imu_to_camera, dtype=np.float64).reshape(3, 1)
            storage.write("translation_imu_to_camera_in_imu_m", vector_imu)
            storage.write("translation_imu_to_camera_in_camera_m",
                          rotation_imu_to_camera @ vector_imu)
            storage.write("imu_camera_center_distance_m", float(np.linalg.norm(vector_imu)))
        storage.write("rotation_rms_deg", float(math.sqrt(np.mean(np.square(residuals)))))
        storage.write("rotation_max_deg", float(max(residuals)))
        storage.write("checkerboard_reprojection_rms_px",
                      float(math.sqrt(np.mean([sample["reprojection_px"] ** 2 for sample in samples]))))
        storage.write("sample_count", len(samples))
        storage.write("orientation_span_deg", float(max(relative_angles, default=0.0)))
        storage.endWriteStruct()
    finally:
        storage.release()


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate camera-to-IMU rotation from static chessboard samples.")
    parser.add_argument("--imu-csv", type=Path, required=True,
                        help="CSV: image,qw,qx,qy,qz; quaternion maps IMU coordinates to world")
    parser.add_argument("--images", type=Path,
                        help="directory for relative image paths (default: CSV directory)")
    parser.add_argument("--camera-calibration", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration" /
                        "camera1_fisheye_1280x720_rectilinear_f400.yaml")
    parser.add_argument("--pattern-cols", type=int, required=True,
                        help="number of checkerboard inner corners along width")
    parser.add_argument("--pattern-rows", type=int, required=True,
                        help="number of checkerboard inner corners along height")
    parser.add_argument("--square-m", type=float, required=True,
                        help="checkerboard square size in metres")
    parser.add_argument("--method", choices=sorted(METHODS), default="park",
                        help="rotation hand-eye solver (default: park)")
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument("--imu-to-camera-m", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="measured vector from IMU centre to camera centre, in IMU axes (m)")
    parser.add_argument("--imu-camera-distance-m", type=float,
                        help="measured centre distance (m); records a constraint only without vector")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration" /
                        "camera1_imu_extrinsics.yaml")
    args = parser.parse_args()
    if args.pattern_cols < 3 or args.pattern_rows < 3 or args.square_m <= 0:
        parser.error("pattern dimensions must be at least 3 and square-m must be positive")
    if args.min_samples < 3:
        parser.error("min-samples must be at least 3")
    if args.imu_camera_distance_m is not None and args.imu_camera_distance_m <= 0:
        parser.error("imu-camera-distance-m must be positive")
    if args.imu_to_camera_m is not None and args.imu_camera_distance_m is not None:
        measured = np.linalg.norm(args.imu_to_camera_m)
        if abs(measured - args.imu_camera_distance_m) > .005:
            parser.error("imu-to-camera-m length differs from imu-camera-distance-m by over 5 mm")

    try:
        camera_matrix, width, height = read_rectified_intrinsics(args.camera_calibration)
        image_directory = args.images or args.imu_csv.parent
        object_points = chessboard_object_points(
            args.pattern_cols, args.pattern_rows, args.square_m)
        samples, rejected = load_samples(
            args.imu_csv, image_directory, object_points,
            (args.pattern_cols, args.pattern_rows), camera_matrix, width, height)
        for row, reason in rejected:
            print("skip CSV row {}: {}".format(row, reason), file=sys.stderr)
        if len(samples) < args.min_samples:
            raise ValueError("only {} valid samples; need at least {}".format(
                len(samples), args.min_samples))
        rotation, residuals, relative_angles = calibrate_rotation(samples, args.method)
        if max(relative_angles, default=0.0) < 20:
            print("warning: orientation span is under 20 degrees; capture more diverse roll, pitch, "
                  "and yaw poses", file=sys.stderr)
        write_result(
            args.output, camera_matrix, width, height, rotation, samples, residuals,
            relative_angles, args.imu_to_camera_m, args.imu_camera_distance_m)
    except (OSError, ValueError, cv2.error) as error:
        print("calibration failed: {}".format(error), file=sys.stderr)
        return 1

    print("wrote {}".format(args.output))
    print("accepted {} samples; rotation RMS {:.3f} deg, maximum {:.3f} deg".format(
        len(samples), math.sqrt(np.mean(np.square(residuals))), max(residuals)))
    print("orientation span {:.1f} deg".format(max(relative_angles, default=0.0)))
    if args.imu_to_camera_m is None:
        print("rotation is calibrated; translation is intentionally unresolved. "
              "Measure IMU-to-camera X,Y,Z in IMU axes for a complete transform.")
    else:
        print("translation vector accepted: [{:.4f}, {:.4f}, {:.4f}] m in IMU axes".format(
            *args.imu_to_camera_m))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
