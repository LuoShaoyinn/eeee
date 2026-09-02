#!/usr/bin/env python3
"""Project many finite upper/lower RANSAC fence-edge fragments into BEV."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import FENCE_HEIGHT_M
from fence_ground_lines import fit_ground_line, pixels_to_plane
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask
from ransac_fence_edges import ransac_segments


def segment_support(edge, segment, inlier_threshold_px):
    start, end, slope, intercept, *_ = segment
    y, x = np.nonzero(edge)
    raw = np.column_stack((x.astype(float), y.astype(float)))
    residual = np.abs(raw[:, 1] - slope * raw[:, 0] - intercept) / np.hypot(slope, 1.0)
    return raw[(raw[:, 0] >= start) & (raw[:, 0] <= end) & (residual < inlier_threshold_px)]


def fit_fragments(edge, plane_z, maximum, height_m, pitch_deg, inlier_threshold_px):
    fragments = []
    for segment in ransac_segments(edge, maximum, threshold_px=inlier_threshold_px):
        support = segment_support(edge, segment, inlier_threshold_px)
        if len(support) < 20:
            continue
        bev = pixels_to_plane(support, height_m, pitch_deg, plane_z)
        centre, direction, _, _, _ = fit_ground_line(bev)
        projection = np.dot(bev - centre, direction)
        start, end = centre + direction * projection.min(), centre + direction * projection.max()
        fragments.append((bev, start, end, segment))
    return fragments


def draw_bev(lower, upper, output):
    scale, margin = 180, 60
    x_min, x_max, y_min, y_max = -1.0, 4.0, -2.5, 2.5
    width, height = round((x_max - x_min) * scale) + 2 * margin, round((y_max - y_min) * scale) + 2 * margin
    image = np.full((height, width, 3), (245, 245, 245), np.uint8)

    def pixel(point):
        x, y = point
        return round(margin + (x - x_min) * scale), round(margin + (y_max - y) * scale)

    for value in range(-1, 5):
        cv2.line(image, pixel((value, y_min)), pixel((value, y_max)), (222, 222, 222), 1)
    for value in range(-2, 3):
        cv2.line(image, pixel((x_min, value)), pixel((x_max, value)), (222, 222, 222), 1)
    cv2.arrowedLine(image, pixel((0, 0)), pixel((.4, 0)), (0, 0, 0), 2, cv2.LINE_AA, 0, .2)
    cv2.putText(image, "camera ground projection", pixel((.05, -.15)), cv2.FONT_HERSHEY_SIMPLEX,
                .38, (0, 0, 0), 1, cv2.LINE_AA)
    palette = ((0, 115, 255), (0, 185, 255), (0, 200, 0), (255, 100, 0), (200, 0, 200), (0, 190, 190))
    for name, fragments, offset in (("lower", lower, 0), ("upper", upper, 3)):
        for index, (points, start, end, _) in enumerate(fragments):
            color = palette[(index + offset) % len(palette)]
            cv2.polylines(image, [np.asarray([pixel(point) for point in points], np.int32)], False, color, 1,
                          cv2.LINE_AA)
            cv2.line(image, pixel(start), pixel(end), color, 3 if name == "lower" else 2, cv2.LINE_AA)
            midpoint = (start + end) / 2
            cv2.putText(image, "{}{}".format(name[0].upper(), index + 1), pixel(midpoint),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
    cv2.putText(image, "lower: thick; upper: thin", (margin, 28), cv2.FONT_HERSHEY_SIMPLEX,
                .5, (0, 0, 0), 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image)


def draw_raw_fit(image, blue, lower_edge, upper_edge, lower, upper, output):
    """Show every finite image-space fit used by the BEV projection."""
    result = image.copy()
    tint = result.copy()
    tint[blue > 0] = (180, 0, 180)
    result = cv2.addWeighted(result, .68, tint, .32, 0)
    result[lower_edge > 0] = (0, 0, 255)
    result[upper_edge > 0] = (0, 255, 0)
    for name, fragments, color in (("L", lower, (0, 140, 255)), ("U", upper, (255, 210, 0))):
        for index, (_, _, _, segment) in enumerate(fragments):
            start, end, slope, intercept, span, _, error = segment
            first = (round(start), round(slope * start + intercept))
            last = (round(end), round(slope * end + intercept))
            cv2.line(result, first, last, color, 2, cv2.LINE_AA)
            cv2.putText(result, "{}{} {}px".format(name, index + 1, round(span)), first,
                        cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--segments", type=int, default=12)
    parser.add_argument("--inlier-threshold-px", type=float, default=1.0,
                        help="perpendicular edge residual required for each RANSAC fragment")
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-bev-ransac.jpg"))
    parser.add_argument("--image-output", type=Path, default=Path("/tmp/fence-ransac-many.jpg"))
    parser.add_argument("--camera-height-m", type=float, default=.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("cannot read {}".format(args.image))
    thresholds = load_thresholds(args.thresholds)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), thresholds["blue_fence"]),
        cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    _, _, _, upper_edge, lower_edge = column_edges(blue, min_pixels=8)
    lower = fit_fragments(lower_edge, 0.0, args.segments, args.camera_height_m,
                          args.pitch_down_deg, args.inlier_threshold_px)
    upper = fit_fragments(upper_edge, FENCE_HEIGHT_M, args.segments,
                          args.camera_height_m, args.pitch_down_deg, args.inlier_threshold_px)
    draw_bev(lower, upper, args.output)
    draw_raw_fit(image, blue, lower_edge, upper_edge, lower, upper, args.image_output)
    print("lower_fragments={} upper_fragments={}".format(len(lower), len(upper)))
    print("wrote {}".format(args.output))
    print("wrote {}".format(args.image_output))


if __name__ == "__main__":
    main()
