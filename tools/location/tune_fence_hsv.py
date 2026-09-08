#!/usr/bin/env python3
"""Tune the blue-fence HSV range on a recorded rectified video."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def noop(_value):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--output", type=Path, default=Path("fence-hsv.json"))
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error("cannot open {}".format(args.video))
    frame_count = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
    defaults = [92, 75, 45, 135, 255, 255]
    if args.output.is_file():
        profile = json.loads(args.output.read_text())
        defaults = [profile[key] for key in
                    ("h_min", "s_min", "v_min", "h_max", "s_max", "v_max")]

    window = "Fence HSV: green=upper orange=lower; S=save Q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    controls = [("frame", frame_count - 1, 0),
                ("h_min", 179, defaults[0]), ("s_min", 255, defaults[1]),
                ("v_min", 255, defaults[2]), ("h_max", 179, defaults[3]),
                ("s_max", 255, defaults[4]), ("v_max", 255, defaults[5])]
    for name, maximum, value in controls:
        cv2.createTrackbar(name, window, value, maximum, noop)

    last_frame = -1
    image = None
    while True:
        frame_index = cv2.getTrackbarPos("frame", window)
        if frame_index != last_frame:
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, image = capture.read()
            if not ok:
                break
            last_frame = frame_index
        values = [cv2.getTrackbarPos(name, window) for name, _, _ in controls[1:]]
        lower = np.array(values[:3], np.uint8)
        upper = np.array(values[3:], np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3)))
        preview = image.copy()
        overlay = image.copy()
        overlay[mask != 0] = (255, 80, 40)
        preview = cv2.addWeighted(preview, .72, overlay, .28, 0)
        for x in range(0, mask.shape[1], 8):
            rows = np.flatnonzero(mask[:, x])
            if len(rows) < 4:
                continue
            top = int(rows[0])
            bottom = int(rows[-1])
            if top > 1:
                cv2.circle(preview, (x, top), 3, (0, 255, 0), -1)
            if bottom < mask.shape[0] - 2:
                cv2.circle(preview, (x, bottom), 3, (0, 165, 255), -1)
        cv2.imshow(window, preview)
        key = cv2.waitKey(20) & 0xff
        if key in (ord("q"), 27):
            break
        if key in (ord("s"), ord("S")):
            profile = dict(zip(
                ("h_min", "s_min", "v_min", "h_max", "s_max", "v_max"), values))
            args.output.write_text(json.dumps(profile, indent=2) + "\n")
            print("saved {}: {}".format(args.output, profile), flush=True)
    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
