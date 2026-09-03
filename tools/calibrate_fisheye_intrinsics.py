#!/usr/bin/env python3
"""Calibrate a raw fisheye camera and write the robot's YAML calibration file."""

import argparse
import glob
import re
from pathlib import Path

import cv2
import numpy as np


def board_points(cols: int, rows: int, square_m: float) -> np.ndarray:
    points = np.zeros((1, cols * rows, 3), np.float64)
    points[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return points


def find_corners(image: np.ndarray, pattern: tuple[int, int]):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return None
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    return corners.reshape(1, -1, 2).astype(np.float64)


def calibrate(objects, image_points, image_size):
    focal = min(image_size) * .5
    camera_matrix = np.array([[focal, 0.0, image_size[0] / 2],
                              [0.0, focal, image_size[1] / 2],
                              [0.0, 0.0, 1.0]], np.float64)
    distortion = np.zeros((4, 1), np.float64)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8)
    rms, camera_matrix, distortion, rotations, translations = cv2.fisheye.calibrate(
        objects, image_points, image_size, camera_matrix, distortion, None, None, flags, criteria)
    errors = []
    for object_points, observed, rotation, translation in zip(objects, image_points, rotations, translations):
        projected, _ = cv2.fisheye.projectPoints(object_points, rotation, translation, camera_matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.square(projected - observed)))))
    return rms, camera_matrix, distortion, np.asarray(errors)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", nargs="+", required=True,
                        help="raw image paths; shell globs are accepted")
    parser.add_argument("--pattern-cols", type=int, required=True)
    parser.add_argument("--pattern-rows", type=int, required=True)
    parser.add_argument("--square-m", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--balance", type=float, default=0.0,
                        help="0 crops black fisheye border; 1 preserves full FOV")
    parser.add_argument("--rectified-focal-px", type=float, default=400.0,
                        help="focal length of the output rectilinear 1280x720 image")
    parser.add_argument("--keep-all", action="store_true",
                        help="do not discard high-error checkerboard views before the final fit")
    args = parser.parse_args()
    if min(args.pattern_cols, args.pattern_rows, args.square_m) <= 0 or not 0 <= args.balance <= 1:
        parser.error("invalid board dimensions, square size, or balance")

    paths = sorted({Path(item) for expression in args.images for item in glob.glob(expression)})
    if not paths:
        parser.error("no image paths matched")
    pattern = (args.pattern_cols, args.pattern_rows)
    object_template = board_points(*pattern, args.square_m)
    objects, image_points, used = [], [], []
    image_size = None
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print("skip unreadable {}".format(path))
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            print("skip wrong size {}".format(path))
            continue
        corners = find_corners(image, pattern)
        if corners is None:
            print("skip board not found {}".format(path))
            continue
        objects.append(object_template.copy())
        image_points.append(corners)
        used.append(path)
    if len(used) < 12:
        parser.error("only {} valid checkerboard views; need at least 12".format(len(used)))

    rejected = []
    # OpenCV refuses a view whose planar pose is numerically singular. Remove
    # only the reported view and retry; all other views still participate.
    while True:
        try:
            rms, camera_matrix, distortion, errors = calibrate(objects, image_points, image_size)
            break
        except cv2.error as error:
            match = re.search(r"input array (\d+)", str(error))
            if match is None or len(used) <= 12:
                raise
            index = int(match.group(1))
            if not 0 <= index < len(used):
                raise
            rejected.append(str(used[index]))
            print("rejected ill-conditioned {}".format(used[index]))
            del objects[index]
            del image_points[index]
            del used[index]
    median = float(np.median(errors))
    # A blurred, partially occluded, or badly localized board should not pull
    # the global fisheye model. Refit once after a conservative view-level trim.
    limit = max(.75, median * 2.5)
    keep = errors <= limit
    if not args.keep_all and np.count_nonzero(keep) >= 12 and not np.all(keep):
        rejected.extend(str(path) for path, retain in zip(used, keep) if not retain)
        objects = [value for value, retain in zip(objects, keep) if retain]
        image_points = [value for value, retain in zip(image_points, keep) if retain]
        used = [value for value, retain in zip(used, keep) if retain]
        rms, camera_matrix, distortion, errors = calibrate(objects, image_points, image_size)
    rectified = np.array([[args.rectified_focal_px, 0.0, image_size[0] / 2],
                          [0.0, args.rectified_focal_px, image_size[1] / 2],
                          [0.0, 0.0, 1.0]], np.float64)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    try:
        storage.write("model", "opencv_fisheye")
        storage.write("projection", "rectilinear")
        storage.write("image_width", image_size[0])
        storage.write("image_height", image_size[1])
        storage.write("K", camera_matrix)
        storage.write("D", distortion)
        storage.write("rectified_K", rectified)
        storage.write("balance", args.balance)
        storage.write("rms_reprojection_px", float(rms))
        storage.write("median_view_reprojection_px", float(np.median(errors)))
        storage.write("valid_view_count", len(used))
        storage.write("rejected_view_count", len(rejected))
        storage.write("checkerboard_inner_cols", args.pattern_cols)
        storage.write("checkerboard_inner_rows", args.pattern_rows)
        storage.write("checkerboard_square_m", args.square_m)
    finally:
        storage.release()
    print("valid_views={} rejected_views={} rms_px={:.4f} median_view_px={:.4f}".format(
        len(used), len(rejected), rms, np.median(errors)))
    for path in rejected:
        print("rejected {}".format(path))
    print("wrote {}".format(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
