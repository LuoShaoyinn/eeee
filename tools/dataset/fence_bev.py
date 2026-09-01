#!/usr/bin/env python3
"""Project blue-fence upper/lower contours into a car-centred bird's-eye view."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import CAMERA_MATRIX, FENCE_HEIGHT_M
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def camera_to_car_rotation(pitch_down_deg):
    pitch = math.radians(pitch_down_deg)
    sine, cosine = math.sin(pitch), math.cos(pitch)
    return np.array([[0.0, -sine, cosine], [-1.0, 0.0, 0.0],
                     [0.0, -cosine, -sine]], dtype=np.float64)


def pixels_to_plane(edge, plane_height_m, camera_height_m, pitch_down_deg):
    ys, xs = np.nonzero(edge)
    rays_camera = np.linalg.solve(
        CAMERA_MATRIX, np.column_stack((xs, ys, np.ones(len(xs)))).T).T
    rays_car = rays_camera @ camera_to_car_rotation(pitch_down_deg).T
    scale = (plane_height_m - camera_height_m) / rays_car[:, 2]
    points = np.column_stack((np.zeros(len(xs)), np.zeros(len(xs)),
                              np.full(len(xs), camera_height_m))) + rays_car * scale[:, None]
    valid = ((scale > 0) & (points[:, 0] > 0.05) & (points[:, 0] < 5.0) &
             (np.abs(points[:, 1]) < 3.0))
    return points[valid, :2]


def bounded_hough_segments(edge, maximum=5):
    """Return long, distinct contour segments without extending them beyond support."""
    detected = cv2.HoughLinesP(edge, 1, np.pi / 180, threshold=35,
                                minLineLength=max(50, edge.shape[1] // 20), maxLineGap=25)
    if detected is None:
        return []
    candidates = []
    for x0, y0, x1, y1 in detected.reshape(-1, 4):
        if x0 == x1:
            continue
        start, end = sorted((float(x0), float(x1)))
        slope = (float(y1) - float(y0)) / (float(x1) - float(x0))
        intercept = float(y0) - slope * float(x0)
        candidates.append((end - start, start, end, slope, intercept))
    chosen = []
    for candidate in sorted(candidates, reverse=True):
        _, start, end, slope, intercept = candidate
        if any(max(0.0, min(end, other[2]) - max(start, other[1])) >= .5 * min(end - start, other[2] - other[1])
               and abs(slope - other[3]) < .03 and abs(intercept - other[4]) < 8 for other in chosen):
            continue
        chosen.append(candidate)
        if len(chosen) == maximum:
            break
    return sorted(chosen, key=lambda item: item[1])


def segments_to_edge(shape, segments):
    edge = np.zeros(shape, dtype=np.uint8)
    for _, start, end, slope, intercept in segments:
        xs = np.arange(round(start), round(end) + 1)
        ys = np.rint(slope * xs + intercept).astype(int)
        valid = (ys >= 0) & (ys < shape[0])
        edge[ys[valid], xs[valid]] = 255
    return edge


def draw_bev(lower, upper, output, scale=250, forward_max_m=4.0, side_extent_m=2.0):
    width, height = round(2 * side_extent_m * scale), round(forward_max_m * scale)
    result = np.full((height, width, 3), 245, dtype=np.uint8)
    for forward in np.arange(.5, forward_max_m, .5):
        cv2.line(result, (0, round((forward_max_m - forward) * scale)),
                 (width - 1, round((forward_max_m - forward) * scale)), (220, 220, 220), 1)
    for left in np.arange(-side_extent_m, side_extent_m + .01, .5):
        x = round((side_extent_m - left) * scale)
        cv2.line(result, (x, 0), (x, height - 1), (220, 220, 220), 1)
    def pixel(points):
        return np.column_stack(((side_extent_m - points[:, 1]) * scale,
                                (forward_max_m - points[:, 0]) * scale)).round().astype(int)
    for points, color in ((lower, (0, 165, 255)), (upper, (0, 255, 0))):
        points = pixel(points)
        visible = ((points[:, 0] >= 0) & (points[:, 0] < width) &
                   (points[:, 1] >= 0) & (points[:, 1] < height))
        for point in points[visible]:
            cv2.circle(result, tuple(point), 1, color, -1)
    cv2.circle(result, (round(side_extent_m * scale), height - 1), 4, (0, 0, 0), -1)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-height-m", type=float, default=.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("cannot read {}".format(args.image))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    thresholds = load_thresholds(args.thresholds)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(hsv, thresholds["blue_fence"]), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)))
    _, _, _, upper_edge, lower_edge = column_edges(blue, min_pixels=8)
    lower_segments = bounded_hough_segments(lower_edge)
    upper_segments = bounded_hough_segments(upper_edge)
    lower = pixels_to_plane(segments_to_edge(blue.shape, lower_segments), 0.0,
                             args.camera_height_m, args.pitch_down_deg)
    upper = pixels_to_plane(segments_to_edge(blue.shape, upper_segments), FENCE_HEIGHT_M,
                             args.camera_height_m, args.pitch_down_deg)
    draw_bev(lower, upper, args.output)
    print("lower_segments={} upper_segments={} lower_points={} upper_points={} wrote={}".format(
        len(lower_segments), len(upper_segments), len(lower), len(upper), args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
