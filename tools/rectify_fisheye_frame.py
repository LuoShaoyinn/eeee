#!/usr/bin/env python3
"""Rectify one Cubie fisheye frame with the versioned camera calibration."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="raw camera image")
    parser.add_argument("output", type=Path, help="rectified output image")
    parser.add_argument("--calibration", type=Path,
                        default=Path("config/camera_fisheye_1280x720.yaml"))
    args = parser.parse_args()

    raw = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if raw is None:
        raise SystemExit(f"cannot read image: {args.input}")
    calibration = cv2.FileStorage(str(args.calibration), cv2.FILE_STORAGE_READ)
    if not calibration.isOpened():
        raise SystemExit(f"cannot open calibration: {args.calibration}")
    camera_matrix = calibration.getNode("K").mat()
    distortion = calibration.getNode("D").mat()
    rectified_matrix = calibration.getNode("rectified_K").mat()
    calibration.release()
    if camera_matrix is None or distortion is None or rectified_matrix is None:
        raise SystemExit("calibration must contain K, D, and rectified_K")

    height, width = raw.shape[:2]
    scale_x = width / 1280.0
    scale_y = height / 720.0
    source_matrix = camera_matrix.astype(np.float64).copy()
    scaled_matrix = rectified_matrix.astype(np.float64).copy()
    for matrix in (source_matrix, scaled_matrix):
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
    map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
        source_matrix, distortion, np.eye(3), scaled_matrix, (width, height), cv2.CV_16SC2)
    rectified = cv2.remap(raw, map_x, map_y, cv2.INTER_LINEAR)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), rectified):
        raise SystemExit(f"cannot write image: {args.output}")
    print(f"rectified {width}x{height}: {args.input} -> {args.output}")


if __name__ == "__main__":
    main()
