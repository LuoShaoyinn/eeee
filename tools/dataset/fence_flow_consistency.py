#!/usr/bin/env python3
"""Check whether local optical flow predicts blue-fence contour motion in a video."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def fence_edges(frame, blue_ranges):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(hsv, blue_ranges), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    _, _, _, upper, lower = column_edges(blue, min_pixels=4)
    return upper, lower


def warp_error(edge, next_edge, flow):
    y, x = np.nonzero(edge)
    if len(x) < 30:
        return None
    vectors = flow[y, x]
    magnitude = np.linalg.norm(vectors, axis=1)
    valid = (magnitude > .03) & (magnitude < 15.0)
    if valid.sum() < 30:
        return None
    x, y, vectors = x[valid], y[valid], vectors[valid]
    distance = cv2.distanceTransform(255 - next_edge, cv2.DIST_L2, 3)
    baseline = distance[y, x]
    warped_x = np.rint(x + vectors[:, 0]).astype(int)
    warped_y = np.rint(y + vectors[:, 1]).astype(int)
    inside = ((warped_x >= 0) & (warped_x < distance.shape[1]) &
              (warped_y >= 0) & (warped_y < distance.shape[0]))
    flowed = distance[warped_y[inside], warped_x[inside]]
    return (float(np.median(baseline[inside])), float(np.median(flowed)),
            float(np.mean(baseline[inside])), float(np.mean(flowed)), vectors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--preview", type=Path, default=Path("/tmp/fence-flow-consistency.jpg"))
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error("cannot open {}".format(args.video))
    ok, first = capture.read()
    if not ok:
        parser.error("cannot read first frame")
    thresholds = load_thresholds(args.thresholds)
    previous = cv2.resize(first, (640, 360), interpolation=cv2.INTER_AREA)
    previous_gray = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
    previous_upper, previous_lower = fence_edges(previous, thresholds["blue_fence"])
    results = []
    preview = None
    for index in range(1, args.frames):
        ok, raw = capture.read()
        if not ok:
            break
        frame = cv2.resize(raw, (640, 360), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(previous_gray, gray, None, .5, 3, 21, 3, 5, 1.2, 0)
        upper, lower = fence_edges(frame, thresholds["blue_fence"])
        for name, source, target in (("upper", previous_upper, upper), ("lower", previous_lower, lower)):
            measurement = warp_error(source, target, flow)
            if measurement is not None:
                median_baseline, median_warped, mean_baseline, mean_warped, vectors = measurement
                results.append((name, median_baseline, median_warped, mean_baseline, mean_warped))
        if preview is None and index > 20:
            preview = frame.copy()
            preview[upper > 0] = (0, 255, 0)
            preview[lower > 0] = (0, 0, 255)
            y, x = np.nonzero(previous_lower)
            for point_x, point_y in zip(x[::12], y[::12]):
                dx, dy = flow[point_y, point_x]
                cv2.arrowedLine(preview, (point_x, point_y),
                                (round(point_x + dx), round(point_y + dy)), (255, 255, 0), 1,
                                cv2.LINE_AA, 0, .25)
        previous, previous_gray = frame, gray
        previous_upper, previous_lower = upper, lower
    capture.release()
    if not results:
        raise SystemExit("no valid contour-flow comparisons")
    for name in ("upper", "lower"):
        values = [(median_baseline, median_warped, mean_baseline, mean_warped)
                  for observed, median_baseline, median_warped, mean_baseline, mean_warped in results
                  if observed == name]
        if values:
            median_baseline, median_warped, mean_baseline, mean_warped = np.median(values, axis=0)
            print("{}: comparisons={} median={:.3f}->{:.3f}px mean={:.3f}->{:.3f}px".format(
                name, len(values), median_baseline, median_warped, mean_baseline, mean_warped))
    if preview is not None:
        cv2.imwrite(str(args.preview), preview)
        print("wrote {}".format(args.preview))


if __name__ == "__main__":
    main()
