#!/usr/bin/env python3
"""Project a rectified camera pixel onto the level ground in the car frame."""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np


def read_mount(path):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open mount calibration: {}".format(path))
    try:
        camera_matrix = storage.getNode("K").mat()
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        mount = storage.getNode("camera_mount")
        camera_height_m = float(mount.getNode("camera_optical_center_height_m").real())
        pitch_down_deg = float(mount.getNode("optical_axis_pitch_down_deg").real())
    finally:
        storage.release()
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise ValueError("mount calibration must contain a 3x3 K matrix")
    if width <= 0 or height <= 0 or camera_height_m <= 0:
        raise ValueError("mount calibration contains invalid dimensions or height")
    return camera_matrix.astype(np.float64), width, height, camera_height_m, pitch_down_deg


def camera_to_car_rotation(pitch_down_deg):
    """Map OpenCV camera axes (right, down, forward) to car axes (forward, left, up)."""
    pitch = math.radians(pitch_down_deg)
    sine, cosine = math.sin(pitch), math.cos(pitch)
    return np.array([
        [0.0, -sine, cosine],
        [-1.0, 0.0, 0.0],
        [0.0, -cosine, -sine],
    ], dtype=np.float64)


def pixel_to_car_ground(pixel_x, pixel_y, camera_matrix, camera_height_m, pitch_down_deg):
    """Return [forward_m, left_m] on z=0, whose origin is below the camera."""
    ray_camera = np.linalg.solve(
        camera_matrix, np.array([pixel_x, pixel_y, 1.0], dtype=np.float64))
    ray_car = camera_to_car_rotation(pitch_down_deg) @ ray_camera
    if ray_car[2] >= -1e-9:
        raise ValueError("pixel ray is at or above the ground-plane horizon")
    scale = -camera_height_m / ray_car[2]
    point_car = np.array([0.0, 0.0, camera_height_m]) + scale * ray_car
    return point_car[:2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float, help="rectified image pixel X, increasing right")
    parser.add_argument("y", type=float, help="rectified image pixel Y, increasing down")
    parser.add_argument("--mount", type=Path,
                        default=Path(__file__).resolve().parents[2] / "config" / "calibration" /
                        "camera1_mount.yaml")
    args = parser.parse_args()
    try:
        camera_matrix, width, height, camera_height_m, pitch_down_deg = read_mount(args.mount)
        if not 0 <= args.x < width or not 0 <= args.y < height:
            parser.error("pixel must be within the {}x{} rectified image".format(width, height))
        forward_m, left_m = pixel_to_car_ground(
            args.x, args.y, camera_matrix, camera_height_m, pitch_down_deg)
    except (OSError, ValueError, cv2.error) as error:
        print("projection failed: {}".format(error), file=sys.stderr)
        return 1
    print("forward_m={:.4f} left_m={:.4f}".format(forward_m, left_m))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
