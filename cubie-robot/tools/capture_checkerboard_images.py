#!/usr/bin/env python3
"""Save rectified checkerboard stills interactively from a V4L2 camera."""

import argparse
import select
import sys
import termios
import tty
from pathlib import Path

import cv2
import numpy as np


def load_rectification(path, width, height):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open calibration {}".format(path))
    try:
        camera_matrix = storage.getNode("K").mat()
        distortion = storage.getNode("D").mat()
        rectified_matrix = storage.getNode("rectified_K").mat()
        calibration_width = int(storage.getNode("image_width").real())
        calibration_height = int(storage.getNode("image_height").real())
    finally:
        storage.release()
    if (camera_matrix is None or distortion is None or rectified_matrix is None or
            calibration_width != width or calibration_height != height):
        raise ValueError("calibration must contain K, D, rectified_K matching capture size")
    return cv2.fisheye.initUndistortRectifyMap(
        camera_matrix, distortion, np.eye(3), rectified_matrix,
        (width, height), cv2.CV_16SC2)


def find_checkerboard(gray, pattern_size):
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    return found, corners


def main():
    parser = argparse.ArgumentParser(
        description="Press Enter to save a rectified checkerboard image; q then Enter exits.")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--pattern-cols", type=int, required=True)
    parser.add_argument("--pattern-rows", type=int, required=True)
    parser.add_argument("--max-images", type=int, default=8)
    parser.add_argument("--calibration", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration" /
                        "camera1_fisheye_1280x720_rectilinear_f400.yaml")
    args = parser.parse_args()
    if min(args.width, args.height, args.fps, args.pattern_cols, args.pattern_rows,
           args.max_images) <= 0:
        parser.error("all dimensions, fps, pattern dimensions, and max-images must be positive")

    try:
        map1, map2 = load_rectification(args.calibration, args.width, args.height)
    except (ValueError, cv2.error) as error:
        print("capture failed: {}".format(error), file=sys.stderr)
        return 1
    args.output_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if not capture.isOpened():
        print("capture failed: cannot open {}".format(args.device), file=sys.stderr)
        return 1

    pattern_size = (args.pattern_cols, args.pattern_rows)
    saved = 0
    print("Camera ready. Put the checkerboard flat on the level floor.")
    print("Press Space to save a detected board; press q to stop.")
    old_settings = None
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        while saved < args.max_images:
            ok, raw = capture.read()
            if not ok or raw.shape[1] != args.width or raw.shape[0] != args.height:
                print("capture failed: invalid camera frame", file=sys.stderr)
                return 1
            rectified = cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if not ready:
                continue
            command = sys.stdin.read(1).lower()
            if command == "q":
                break
            if command != " ":
                continue
            gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
            found, _ = find_checkerboard(gray, pattern_size)
            if not found:
                print("checkerboard not found; reposition it and press Enter again")
                continue
            saved += 1
            output = args.output_dir / "floor-{:02d}.png".format(saved)
            if not cv2.imwrite(str(output), rectified):
                print("capture failed: cannot write {}".format(output), file=sys.stderr)
                return 1
            print("saved {}".format(output))
    finally:
        capture.release()
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    print("saved {} image(s)".format(saved))
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())
