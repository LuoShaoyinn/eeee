#!/usr/bin/env python3
"""Emit calibrated A733 YOLO detections in robotbrain's frame protocol.

This bridge keeps camera correction explicit while reusing the already
validated A733 YOLO26 executable.  It is intentionally a supervised V1 path:
it never commands motors.  Pipe its output to robotbrain only after checking
the emitted frames first.
"""

import argparse
import os
import re
import subprocess
import sys
import time

import cv2
import numpy as np


LABELS = {
    "yellow_cylinder": "yellow",
    "red_cube": "red",
    "other_robot": "other_robot",
    "home": "home",
}
DETECTION = re.compile(
    r"^(yellow_cylinder|red_cube|other_robot|home)\s+([0-9.]+)%\s+\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]$")


def load_maps(path: str, width: int, height: int):
    calibration = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not calibration.isOpened():
        raise RuntimeError(f"cannot open calibration: {path}")
    source_k = calibration.getNode("K").mat()
    distortion = calibration.getNode("D").mat()
    rectified_k = calibration.getNode("rectified_K").mat()
    calibration.release()
    if source_k is None or distortion is None or rectified_k is None:
        raise RuntimeError("calibration must contain K, D, and rectified_K")
    source_k = source_k.astype(np.float64)
    distortion = distortion.astype(np.float64)
    rectified_k = rectified_k.astype(np.float64)
    scale_x, scale_y = width / 1280.0, height / 720.0
    for matrix in (source_k, rectified_k):
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
    return cv2.fisheye.initUndistortRectifyMap(
        source_k, distortion, np.eye(3), rectified_k, (width, height), cv2.CV_16SC2)


def parse_detections(output: str, width: int, height: int):
    detections = []
    for line in output.splitlines():
        match = DETECTION.match(line.strip())
        if not match:
            continue
        label, score, left, top, right, bottom = match.groups()
        left, top, right, bottom = map(int, (left, top, right, bottom))
        detections.append((LABELS[label], float(score) / 100.0, left, top, right, bottom))
    return detections


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--calibration", default="config/camera_fisheye_1280x720.yaml")
    parser.add_argument("--yolo-dir", default="/home/radxa/yolo26n-a733")
    parser.add_argument("--yolo-bin", default="./yolo26_demo_640_a733",
                        help="A733 executable matching the selected model's decoder")
    parser.add_argument("--model", default="/home/radxa/cubie-robot/models/official_yolo26n_640x384_rgbfix_rebuild_a733.nb",
                        help="RGB-corrected 640x384 A733 INT8 model with two 4x5040 outputs")
    parser.add_argument("--minimum-blue-pixels", type=int, default=60)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means stream until interrupted")
    parser.add_argument("--frame-path", default="/tmp/robotvision-rectified.jpg")
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 20)
    if not capture.isOpened():
        raise SystemExit(f"cannot open camera: {args.camera}")
    maps = None
    frames = 0
    try:
        while not args.max_frames or frames < args.max_frames:
            ok, raw = capture.read()
            if not ok or raw is None:
                raise RuntimeError("camera capture failed")
            if maps is None:
                maps = load_maps(args.calibration, raw.shape[1], raw.shape[0])
            rectified = cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR)
            hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
            blue_pixels = int(cv2.countNonZero(cv2.inRange(hsv, (92, 75, 45), (135, 255, 255))))
            environment = os.environ.copy()
            environment["LD_LIBRARY_PATH"] = args.yolo_dir + ":" + environment.get("LD_LIBRARY_PATH", "")
            # The NPU runner accepts a file path.  Write the corrected, unannotated
            # image first: otherwise it would consume the preceding annotated frame.
            inference_path = args.frame_path + ".inference.jpg"
            if not cv2.imwrite(inference_path, rectified):
                raise RuntimeError(f"cannot write corrected inference frame: {inference_path}")
            result = subprocess.run(
                [args.yolo_bin, "-nb", args.model, "-i", inference_path, "-l", "1"],
                cwd=args.yolo_dir, env=environment, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False)
            if result.returncode != 0:
                raise RuntimeError("A733 YOLO failed: " + result.stdout[-500:])
            detections = parse_detections(result.stdout, rectified.shape[1], rectified.shape[0])
            annotated = rectified.copy()
            frame = ["1" if blue_pixels >= args.minimum_blue_pixels else "0", "0"]
            for label, confidence, left, top, right, bottom in detections:
                center_x = max(0.0, min(1.0, (left + right) * .5 / rectified.shape[1]))
                bottom_y = max(0.0, min(1.0, bottom / rectified.shape[0]))
                frame.extend((label, f"{confidence:.3f}", f"{center_x:.3f}", f"{bottom_y:.3f}"))
                cv2.rectangle(annotated, (left, top), (right, bottom), (255, 170, 0), 2)
                cv2.putText(annotated, f"{label} {confidence:.0%}", (left, max(18, top - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 170, 0), 2)
            temporary_path = args.frame_path + ".tmp.jpg"
            if not cv2.imwrite(temporary_path, annotated):
                raise RuntimeError(f"cannot write rectified frame: {temporary_path}")
            os.replace(temporary_path, args.frame_path)
            print(" ".join(frame), flush=True)
            print(f"robotvision: frame={frames} blue={blue_pixels} detections={len(detections)}",
                  file=sys.stderr, flush=True)
            frames += 1
    finally:
        capture.release()


if __name__ == "__main__":
    main()
