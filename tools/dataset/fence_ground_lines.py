#!/usr/bin/env python3
"""Back-project finite lower fence-edge RANSAC segments onto a flat ground plane."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import CAMERA_MATRIX, FENCE_HEIGHT_M
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask
from ransac_fence_edges import ransac_segments


def camera_to_car_rotation(pitch_down_deg: float) -> np.ndarray:
    pitch = math.radians(pitch_down_deg)
    sine, cosine = math.sin(pitch), math.cos(pitch)
    # Car coordinates: +X forward, +Y left, +Z up.
    return np.array([[0.0, -sine, cosine], [-1.0, 0.0, 0.0],
                     [0.0, -cosine, -sine]], dtype=float)


def pixels_to_plane(pixels: np.ndarray, height_m: float, pitch_down_deg: float,
                    plane_z_m: float) -> np.ndarray:
    normalized = np.column_stack(((pixels[:, 0] - CAMERA_MATRIX[0, 2]) / CAMERA_MATRIX[0, 0],
                                  (pixels[:, 1] - CAMERA_MATRIX[1, 2]) / CAMERA_MATRIX[1, 1],
                                  np.ones(len(pixels))))
    directions = normalized @ camera_to_car_rotation(pitch_down_deg).T
    scale = (plane_z_m - height_m) / directions[:, 2]
    return directions[:, :2] * scale[:, None]


def pixels_to_ground(pixels: np.ndarray, height_m: float, pitch_down_deg: float) -> np.ndarray:
    return pixels_to_plane(pixels, height_m, pitch_down_deg, 0.0)


def fit_ground_line(points: np.ndarray):
    centre = np.mean(points, axis=0)
    _, _, vectors = np.linalg.svd(points - centre, full_matrices=False)
    direction = vectors[0]
    if direction[0] < 0:
        direction = -direction
    normal = np.array([-direction[1], direction[0]])
    distance = float(np.dot(normal, centre))
    angle = math.degrees(math.atan2(direction[1], direction[0]))
    return centre, direction, normal, distance, angle


def draw_bev(lines, output: Path):
    scale, margin = 180, 60
    # Show 1 m behind and 4 m in front of the camera ground projection.
    x_min, x_max, y_min, y_max = -1.0, 4.0, -2.5, 2.5
    width, height = round((x_max - x_min) * scale) + 2 * margin, round((y_max - y_min) * scale) + 2 * margin
    image = np.full((height, width, 3), (245, 245, 245), np.uint8)

    def pixel(point):
        x, y = point
        return (round(margin + (x - x_min) * scale), round(margin + (y_max - y) * scale))

    for value in np.arange(math.ceil(x_min), math.floor(x_max) + 1):
        cv2.line(image, pixel((value, y_min)), pixel((value, y_max)), (220, 220, 220), 1)
    for value in np.arange(math.ceil(y_min), math.floor(y_max) + 1):
        cv2.line(image, pixel((x_min, value)), pixel((x_max, value)), (220, 220, 220), 1)
    cv2.arrowedLine(image, pixel((0, 0)), pixel((.45, 0)), (0, 0, 0), 2, cv2.LINE_AA, 0, .18)
    cv2.putText(image, "camera ground projection", pixel((.05, -.13)), cv2.FONT_HERSHEY_SIMPLEX,
                .38, (0, 0, 0), 1, cv2.LINE_AA)
    colors = ((0, 165, 255), (0, 200, 0), (255, 80, 0))
    for index, (points, centre, direction, _, angle) in enumerate(lines):
        color = colors[index % len(colors)]
        cv2.polylines(image, [np.asarray([pixel(point) for point in points], np.int32)], False, color, 2,
                      cv2.LINE_AA)
        start, end = centre - direction * 2.5, centre + direction * 2.5
        cv2.line(image, pixel(start), pixel(end), color, 1, cv2.LINE_AA)
        cv2.putText(image, "L{} {:.1f} deg".format(index + 1, angle), pixel(centre),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image)


def draw_image_fit(image, blue, edge, segments, output: Path, edge_name: str):
    """Render the exact selected-edge samples and RANSAC supports used for BEV."""
    result = image.copy()
    tint = result.copy()
    tint[blue > 0] = (190, 0, 190)
    result = cv2.addWeighted(result, .68, tint, .32, 0)
    result[edge > 0] = (0, 0, 255)
    colors = ((0, 165, 255), (0, 220, 0), (255, 80, 0))
    for index, (start, end, slope, intercept, span, count, error) in enumerate(segments):
        color = colors[index % len(colors)]
        # The outer physical walls are only visible at opposite image sides.
        # Show their infinite-image regressions separately from observed support.
        if index in (0, len(segments) - 1):
            full_first = (0, round(intercept))
            full_last = (image.shape[1] - 1, round(slope * (image.shape[1] - 1) + intercept))
            cv2.line(result, full_first, full_last, color, 1, cv2.LINE_AA)
        first = (round(start), round(slope * start + intercept))
        last = (round(end), round(slope * end + intercept))
        cv2.line(result, first, last, color, 3, cv2.LINE_AA)
        cv2.putText(result, "{} L{}: {}px, {:.1f}px".format(edge_name, index + 1, round(span), error), first,
                    cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-ground-lines.jpg"))
    parser.add_argument("--image-output", type=Path, default=Path("/tmp/fence-ground-image-fit.jpg"))
    parser.add_argument("--camera-height-m", type=float, default=.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    parser.add_argument("--max-occlusion-gap", type=int, default=12,
                        help="maximum missing-x interval allowed within one RANSAC support")
    parser.add_argument("--edge", choices=("lower", "upper"), default="lower")
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("cannot read {}".format(args.image))
    thresholds = load_thresholds(args.thresholds)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), thresholds["blue_fence"]),
        cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    _, _, _, upper, lower = column_edges(blue, min_pixels=8)
    edge = lower if args.edge == "lower" else upper
    plane_z_m = 0.0 if args.edge == "lower" else FENCE_HEIGHT_M
    segments = ransac_segments(edge, 3, max_gap=args.max_occlusion_gap)
    draw_image_fit(image, blue, edge, segments, args.image_output, args.edge)
    lines = []
    y, x = np.nonzero(edge)
    raw = np.column_stack((x.astype(float), y.astype(float)))
    for start, end, slope, intercept, *_ in segments:
        residual = np.abs(raw[:, 1] - slope * raw[:, 0] - intercept) / np.hypot(slope, 1.0)
        supported = raw[(raw[:, 0] >= start) & (raw[:, 0] <= end) & (residual < 3.0)]
        ground = pixels_to_plane(supported, args.camera_height_m, args.pitch_down_deg, plane_z_m)
        centre, direction, normal, distance, angle = fit_ground_line(ground)
        lines.append((ground, centre, direction, distance, angle))
    draw_bev(lines, args.output)
    for index, (_, centre, direction, distance, angle) in enumerate(lines):
        print("{} L{}: direction_deg={:.2f} normal_distance_m={:.3f} point=({:.3f}, {:.3f})".format(
            args.edge,
            index + 1, angle, distance, centre[0], centre[1]))
    print("wrote {}".format(args.output))
    print("wrote {}".format(args.image_output))


if __name__ == "__main__":
    main()
