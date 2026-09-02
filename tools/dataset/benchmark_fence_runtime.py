#!/usr/bin/env python3
"""Benchmark the CPU-only fence-localization front end on a recorded video."""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def fence_measurement(frame: np.ndarray, blue_ranges: list[list[list[int]]]) -> tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = make_mask(hsv, blue_ranges)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = retain_fence_components(mask)
    _, _, _, upper, lower = column_edges(mask, min_pixels=3)
    upper_dt = cv2.distanceTransform(255 - upper, cv2.DIST_L2, 3)
    lower_dt = cv2.distanceTransform(255 - lower, cv2.DIST_L2, 3)
    return upper_dt, lower_dt


def proposal_batch(upper_dt: np.ndarray, lower_dt: np.ndarray, candidates: int) -> None:
    """Represent candidate scoring: 4 walls, two height edges, 64 points/wall.

    The real matcher will replace this fixed image sample set with projected map
    points. The distance-transform lookup and robust reduction cost is identical.
    """
    height, width = upper_dt.shape
    sample_count = 4 * 2 * 64
    rng = np.random.default_rng(7)
    x = rng.integers(0, width, size=(candidates, sample_count))
    y = rng.integers(0, height, size=(candidates, sample_count))
    transform = np.concatenate((upper_dt[None], lower_dt[None]), axis=0)
    edge = np.arange(sample_count)[None, :] & 1
    scores = np.minimum(transform[edge, y, x], 12.0).mean(axis=1)
    # Keep NumPy from eliding the calculation in alternative implementations.
    if not np.isfinite(scores).all():
        raise RuntimeError("non-finite proposal score")


def percentile_ms(values: list[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values), percentile) * 1000.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=180)
    parser.add_argument("--candidates", type=int, default=1575,
                        help="local 15 x 15 x 7 pose grid")
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error("cannot open {}".format(args.video))
    thresholds = load_thresholds(args.thresholds)
    decode_s, resize_s, measurement_s, proposal_s, total_s = [], [], [], [], []
    processed = 0
    while processed < args.frames:
        start = time.perf_counter()
        ok, raw = capture.read()
        decoded = time.perf_counter()
        if not ok:
            break
        frame = cv2.resize(raw, (args.width, args.height), interpolation=cv2.INTER_AREA)
        resized = time.perf_counter()
        upper_dt, lower_dt = fence_measurement(frame, thresholds["blue_fence"])
        measured = time.perf_counter()
        proposal_batch(upper_dt, lower_dt, args.candidates)
        complete = time.perf_counter()
        decode_s.append(decoded - start)
        resize_s.append(resized - decoded)
        measurement_s.append(measured - resized)
        proposal_s.append(complete - measured)
        total_s.append(complete - start)
        processed += 1
    capture.release()
    if not processed:
        raise SystemExit("no video frames processed")
    print("frames={} resolution={}x{} candidates={}".format(
        processed, args.width, args.height, args.candidates))
    for name, values in (("video decode", decode_s), ("resize", resize_s),
                         ("mask+edges+DT", measurement_s),
                         ("proposal lookups", proposal_s), ("total", total_s)):
        print("{:>16}: mean={:6.2f} ms  p95={:6.2f} ms".format(
            name, np.mean(values) * 1000.0, percentile_ms(values, 95)))
    print("pipeline throughput: {:.1f} Hz".format(1.0 / np.mean(total_s)))


if __name__ == "__main__":
    main()
