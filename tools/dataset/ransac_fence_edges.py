#!/usr/bin/env python3
"""Fit finite fence-edge segments with RANSAC ranked by continuous support."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def longest_run(xs: np.ndarray, max_gap: int) -> tuple[int, int, int]:
    """Return index range and pixel span for the longest near-continuous x run."""
    if not len(xs):
        return 0, 0, 0
    breaks = np.flatnonzero(np.diff(xs) > max_gap) + 1
    starts = np.r_[0, breaks]
    ends = np.r_[breaks, len(xs)]
    spans = xs[ends - 1] - xs[starts]
    index = int(np.argmax(spans))
    return int(starts[index]), int(ends[index]), int(spans[index])


def best_segment(points: np.ndarray, rng: np.random.Generator, trials: int,
                 threshold_px: float, max_gap: int):
    """Find a line whose *single longest* inlier run is maximal.

    This differs from a global least-squares/Hough score: unsupported line
    extensions and disconnected blue objects cannot increase the segment score.
    """
    if len(points) < 24:
        return None
    best = None
    for _ in range(trials):
        first, second = points[rng.choice(len(points), 2, replace=False)]
        dx = second[0] - first[0]
        if abs(dx) < 8:
            continue
        slope = (second[1] - first[1]) / dx
        intercept = first[1] - slope * first[0]
        residual = np.abs(points[:, 1] - slope * points[:, 0] - intercept) / np.hypot(slope, 1.0)
        inlier_indices = np.flatnonzero(residual <= threshold_px)
        if len(inlier_indices) < 20:
            continue
        ordered = inlier_indices[np.argsort(points[inlier_indices, 0])]
        start, end, span = longest_run(points[ordered, 0], max_gap)
        support = ordered[start:end]
        if len(support) < 20 or span < 70:
            continue
        # Refine only against the continuous run, then recompute its support.
        slope, intercept = np.polyfit(points[support, 0], points[support, 1], 1)
        residual = np.abs(points[:, 1] - slope * points[:, 0] - intercept) / np.hypot(slope, 1.0)
        inlier_indices = np.flatnonzero(residual <= threshold_px)
        ordered = inlier_indices[np.argsort(points[inlier_indices, 0])]
        start, end, span = longest_run(points[ordered, 0], max_gap)
        support = ordered[start:end]
        candidate = (span, len(support), float(np.mean(residual[support])), slope, intercept, support)
        if best is None or candidate[:3] > best[:3]:
            best = candidate
    return best


def ransac_segments(edge: np.ndarray, maximum: int, trials: int = 1200,
                    threshold_px: float = 2.5, max_gap: int = 12):
    y, x = np.nonzero(edge)
    remaining = np.column_stack((x.astype(float), y.astype(float)))
    rng = np.random.default_rng(12345)
    segments = []
    for _ in range(maximum):
        result = best_segment(remaining, rng, trials, threshold_px, max_gap)
        if result is None:
            break
        span, count, mean_error, slope, intercept, support = result
        segment_points = remaining[support]
        segments.append((segment_points[:, 0].min(), segment_points[:, 0].max(), slope, intercept,
                         span, count, mean_error))
        residual = np.abs(remaining[:, 1] - slope * remaining[:, 0] - intercept) / np.hypot(slope, 1.0)
        # Remove this actual finite support, but retain disconnected collinear sections.
        start, end = segment_points[:, 0].min(), segment_points[:, 0].max()
        remove = (residual <= threshold_px) & (remaining[:, 0] >= start) & (remaining[:, 0] <= end)
        remaining = remaining[~remove]
    return sorted(segments)


def draw_segments(image: np.ndarray, segments, color, label):
    for index, (start, end, slope, intercept, span, count, error) in enumerate(segments):
        first = (round(start), round(slope * start + intercept))
        last = (round(end), round(slope * end + intercept))
        cv2.line(image, first, last, color, 3, cv2.LINE_AA)
        cv2.putText(image, "{}{} {}px".format(label, index + 1, round(span)), first,
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-ransac.jpg"))
    parser.add_argument("--segments", type=int, default=3)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("cannot read {}".format(args.image))
    thresholds = load_thresholds(args.thresholds)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(hsv, thresholds["blue_fence"]), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    _, _, _, upper, lower = column_edges(blue, min_pixels=8)
    upper_segments = ransac_segments(upper, args.segments)
    lower_segments = ransac_segments(lower, args.segments)
    result = image.copy()
    result[blue > 0] = (180, 0, 180)
    result = cv2.addWeighted(image, .62, result, .38, 0)
    draw_segments(result, upper_segments, (0, 255, 0), "U")
    draw_segments(result, lower_segments, (0, 165, 255), "L")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), result)
    for name, segments in (("upper", upper_segments), ("lower", lower_segments)):
        print("{} segments:".format(name))
        for start, end, slope, _, span, count, error in segments:
            print("  x={:.0f}..{:.0f} span={:.0f}px inliers={} slope={:.4f} mean_error={:.2f}px".format(
                start, end, span, count, slope, error))
    print("wrote {}".format(args.output))


if __name__ == "__main__":
    main()
