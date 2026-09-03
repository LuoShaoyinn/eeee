#!/usr/bin/env python3
"""Capture diverse raw checkerboard views for fisheye intrinsic calibration."""

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np


def detect(gray: np.ndarray, pattern: tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return None
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    return corners.reshape(-1, 2)


def descriptor(corners: np.ndarray, cols: int, image_shape: tuple[int, int]) -> np.ndarray:
    height, width = image_shape
    minimum, maximum = corners.min(axis=0), corners.max(axis=0)
    centre = (minimum + maximum) * .5 / np.array([width, height])
    area = float(np.prod(maximum - minimum) / (width * height))
    vector = corners[cols - 1] - corners[0]
    angle = math.atan2(float(vector[1]), float(vector[0])) / math.pi
    return np.array([centre[0], centre[1], area, angle])


def novel(candidate: np.ndarray, accepted: list[np.ndarray]) -> bool:
    if not accepted:
        return True
    for previous in accepted:
        centre_delta = np.linalg.norm(candidate[:2] - previous[:2])
        area_delta = abs(math.log(max(candidate[2], 1e-6) / max(previous[2], 1e-6)))
        angle_delta = abs(candidate[3] - previous[3])
        angle_delta = min(angle_delta, 2.0 - angle_delta)
        if centre_delta < .10 and area_delta < .22 and angle_delta < .10:
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--pattern-cols", type=int, required=True,
                        help="inner-corner count, not block count")
    parser.add_argument("--pattern-rows", type=int, required=True,
                        help="inner-corner count, not block count")
    parser.add_argument("--max-images", type=int, default=30)
    parser.add_argument("--timeout-s", type=float, default=180)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps, args.pattern_cols, args.pattern_rows,
           args.max_images, args.timeout_s) <= 0:
        parser.error("dimensions and limits must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if not capture.isOpened():
        raise RuntimeError("cannot open {}".format(args.device))

    accepted: list[np.ndarray] = []
    pattern = (args.pattern_cols, args.pattern_rows)
    deadline = time.monotonic() + args.timeout_s
    last_saved = 0.0
    print("Move and tilt the board across centre and all image edges; capture ends after "
          "{} diverse poses or {:.0f}s.".format(args.max_images, args.timeout_s), flush=True)
    try:
        while len(accepted) < args.max_images and time.monotonic() < deadline:
            ok, frame = capture.read()
            if not ok or frame.shape[:2] != (args.height, args.width):
                continue
            corners = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), pattern)
            if corners is None or time.monotonic() - last_saved < .6:
                continue
            candidate = descriptor(corners, args.pattern_cols, frame.shape[:2])
            if not novel(candidate, accepted):
                continue
            accepted.append(candidate)
            output = args.output_dir / "raw-{:03d}.png".format(len(accepted))
            if not cv2.imwrite(str(output), frame):
                raise RuntimeError("cannot write {}".format(output))
            last_saved = time.monotonic()
            print("saved {} centre=({:.2f},{:.2f}) area={:.3f}".format(
                output.name, candidate[0], candidate[1], candidate[2]), flush=True)
    finally:
        capture.release()
    print("saved {} diverse raw image(s)".format(len(accepted)), flush=True)
    return 0 if len(accepted) >= 12 else 1


if __name__ == "__main__":
    raise SystemExit(main())
