#!/usr/bin/env python3
"""Project every ordered blue-fence boundary pixel into connected BEV curves."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from estimate_fence_pose import CAMERA_MATRIX, FENCE_HEIGHT_M
from fence_ground_lines import camera_to_car_rotation
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def project_curve(pixels, height_m, pitch_deg, roll_deg, plane_z):
    normalized = np.column_stack(((pixels[:, 0] - CAMERA_MATRIX[0, 2]) / CAMERA_MATRIX[0, 0],
                                  (pixels[:, 1] - CAMERA_MATRIX[1, 2]) / CAMERA_MATRIX[1, 1],
                                  np.ones(len(pixels))))
    roll = np.deg2rad(roll_deg)
    optical_roll = np.array([[np.cos(roll), -np.sin(roll), 0.0],
                             [np.sin(roll), np.cos(roll), 0.0],
                             [0.0, 0.0, 1.0]])
    directions = normalized @ (camera_to_car_rotation(pitch_deg) @ optical_roll).T
    scale = (plane_z - height_m) / directions[:, 2]
    points = directions[:, :2] * scale[:, None]
    return points, np.isfinite(points).all(axis=1) & (scale > 0)


def draw_curve(image, points, valid, color, pixel, label):
    chunks, start = [], None
    for index, okay in enumerate(valid):
        if okay and start is None:
            start = index
        elif not okay and start is not None:
            chunks.append(points[start:index])
            start = None
    if start is not None:
        chunks.append(points[start:])
    for chunk in chunks:
        projected = np.asarray([pixel(point) for point in chunk], np.int32)
        if len(projected) > 1:
            cv2.polylines(image, [projected], False, color, 2, cv2.LINE_AA)
        for point in projected[::12]:
            cv2.circle(image, tuple(point), 1, color, -1, cv2.LINE_AA)
    if chunks:
        projected = np.asarray([pixel(point) for point in max(chunks, key=len)], np.int32)
        cv2.putText(image, label, tuple(projected[len(projected) // 2]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, color, 1, cv2.LINE_AA)


def draw_detected_image(source, blue, xs, upper_y, lower_y, output):
    """Draw the exact column-wise boundaries used before BEV projection."""
    result = source.copy()
    tint = result.copy()
    tint[blue > 0] = (180, 0, 180)
    result = cv2.addWeighted(result, .68, tint, .32, 0)
    upper = np.column_stack((xs, upper_y)).round().astype(np.int32)
    lower = np.column_stack((xs, lower_y)).round().astype(np.int32)
    cv2.polylines(result, [upper], False, (255, 210, 0), 2, cv2.LINE_AA)
    cv2.polylines(result, [lower], False, (0, 140, 255), 2, cv2.LINE_AA)
    cv2.putText(result, "upper detected edge", tuple(upper[len(upper) // 2]), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (255, 210, 0), 1, cv2.LINE_AA)
    cv2.putText(result, "lower detected edge", tuple(lower[len(lower) // 2] + (0, 20)), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (0, 140, 255), 1, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-bev-curve.jpg"))
    parser.add_argument("--image-output", type=Path, default=Path("/tmp/fence-detected-edge.jpg"))
    parser.add_argument("--camera-height-m", type=float, default=.1311825723)
    parser.add_argument("--pitch-down-deg", type=float, default=30.1132498)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--upper-edge-height-m", type=float, default=FENCE_HEIGHT_M,
                        help="height of the visible blue upper boundary, not necessarily fence outer height")
    args = parser.parse_args()
    source = cv2.imread(str(args.image))
    if source is None:
        parser.error("cannot read {}".format(args.image))
    thresholds = load_thresholds(args.thresholds)
    blue = retain_fence_components(cv2.morphologyEx(
        make_mask(cv2.cvtColor(source, cv2.COLOR_BGR2HSV), thresholds["blue_fence"]),
        cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    xs, upper_y, lower_y, _, _ = column_edges(blue, min_pixels=8)
    draw_detected_image(source, blue, xs, upper_y, lower_y, args.image_output)
    lower, lower_valid = project_curve(np.column_stack((xs, lower_y)), args.camera_height_m,
                                       args.pitch_down_deg, args.roll_deg, 0.0)
    upper, upper_valid = project_curve(np.column_stack((xs, upper_y)), args.camera_height_m,
                                       args.pitch_down_deg, args.roll_deg, args.upper_edge_height_m)

    scale, margin = 180, 60
    x_min, x_max, y_min, y_max = -1.0, 4.0, -2.5, 2.5
    width, height = round((x_max - x_min) * scale) + 2 * margin, round((y_max - y_min) * scale) + 2 * margin
    result = np.full((height, width, 3), (245, 245, 245), np.uint8)

    def pixel(point):
        x, y = point
        return round(margin + (x - x_min) * scale), round(margin + (y_max - y) * scale)

    for value in range(-1, 5):
        cv2.line(result, pixel((value, y_min)), pixel((value, y_max)), (222, 222, 222), 1)
    for value in range(-2, 3):
        cv2.line(result, pixel((x_min, value)), pixel((x_max, value)), (222, 222, 222), 1)
    cv2.arrowedLine(result, pixel((0, 0)), pixel((.4, 0)), (0, 0, 0), 2, cv2.LINE_AA, 0, .2)
    draw_curve(result, lower, lower_valid, (0, 100, 255), pixel, "lower z=0")
    draw_curve(result, upper, upper_valid, (255, 190, 0), pixel,
               "upper z={:.3f}".format(args.upper_edge_height_m))
    cv2.putText(result, "all ordered valid edge pixels; curve bridges HSV gaps", (margin, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1, cv2.LINE_AA)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), result)
    print("lower_points={}/{} upper_points={}/{}".format(
        int(lower_valid.sum()), len(lower), int(upper_valid.sum()), len(upper)))
    print("wrote {}".format(args.output))
    print("wrote {}".format(args.image_output))


if __name__ == "__main__":
    main()
