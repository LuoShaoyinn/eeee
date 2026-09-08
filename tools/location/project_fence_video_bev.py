#!/usr/bin/env python3
"""Project complete blue-fence image boundaries into robot-relative BEV."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.interpolate import RBFInterpolator
from scipy.spatial import cKDTree


def load_blue_ranges(path):
    profile = json.loads(path.read_text(encoding="utf-8"))
    ranges = profile.get("blue_fence", [])
    if not ranges:
        raise ValueError("{} has no blue_fence ranges".format(path))
    return [(np.asarray(low, np.uint8), np.asarray(high, np.uint8))
            for low, high in ranges]


def fence_mask(image, ranges, minimum_component_area):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape[:2], np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    count, labels, statistics, _ = cv2.connectedComponentsWithStats(mask, 8)
    retained = np.zeros_like(mask)
    for label in range(1, count):
        if statistics[label, cv2.CC_STAT_AREA] >= minimum_component_area:
            retained[labels == label] = 255
    return retained


def boundary_pixels(mask, minimum_span):
    upper, lower = [], []
    for x in range(mask.shape[1]):
        rows = np.flatnonzero(mask[:, x])
        if len(rows) == 0 or rows[-1] - rows[0] < minimum_span:
            continue
        upper.append((float(x), float(rows[0])))
        lower.append((float(x), float(rows[-1])))
    return np.asarray(upper, np.float64), np.asarray(lower, np.float64)


def image_to_robot(matrix, pixels, height):
    if not len(pixels):
        return np.empty((0, 2), np.float64)
    homography = np.column_stack((matrix[:, 0], matrix[:, 1],
                                  height * matrix[:, 2] + matrix[:, 3]))
    homogeneous_pixels = np.column_stack((pixels, np.ones(len(pixels))))
    robot = np.linalg.solve(homography, homogeneous_pixels.T).T
    valid = np.abs(robot[:, 2]) > 1e-9
    robot[valid] /= robot[valid, 2:3]
    return robot[valid, :2]


def filter_robot_points(points, forward_min, forward_max, lateral_limit):
    if not len(points):
        return points
    valid = (np.isfinite(points).all(axis=1) &
             (points[:, 0] >= forward_min) & (points[:, 0] <= forward_max) &
             (np.abs(points[:, 1]) <= lateral_limit))
    return points[valid]


def load_residual_warp(path):
    if path is None:
        return None, None
    data = np.load(path)
    interpolator = RBFInterpolator(
        data["control_pixels"], data["control_displacements"],
        kernel="thin_plate_spline", smoothing=float(data["smoothing"]), degree=1)
    return interpolator, float(data["maximum_displacement"])


def correct_pixels(pixels, interpolator, maximum):
    if interpolator is None or not len(pixels):
        return pixels
    displacement = interpolator(pixels)
    lengths = np.linalg.norm(displacement, axis=1)
    displacement *= np.minimum(1.0, maximum / np.maximum(lengths, 1e-9))[:, None]
    return pixels + displacement


def correct_radial_pixels(pixels, coefficients, matrix):
    if coefficients is None or not len(pixels):
        return pixels
    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    x = (pixels[:, 0] - cx) / fx
    y = (pixels[:, 1] - cy) / fy
    r2 = x * x + y * y
    k1, k2, p1, p2 = coefficients
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    corrected_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    corrected_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return np.column_stack((fx * corrected_x + cx, fy * corrected_y + cy))


def symmetric_cloud_distance(first, second):
    if not len(first) or not len(second):
        return np.empty(0, np.float64)
    first_to_second = cKDTree(second).query(first, workers=-1)[0]
    second_to_first = cKDTree(first).query(second, workers=-1)[0]
    return np.concatenate((first_to_second, second_to_first))


def draw_bev(upper, lower, timestamp, forward_min, forward_max, lateral_limit):
    width, height = 1000, 760
    margin = 64
    canvas = np.full((height, width, 3), 248, np.uint8)

    def map_points(points):
        x = margin + (lateral_limit - points[:, 1]) * (
            (width - 2 * margin) / (2 * lateral_limit))
        y = height - margin - (points[:, 0] - forward_min) * (
            (height - 2 * margin) / (forward_max - forward_min))
        return np.rint(np.column_stack((x, y))).astype(np.int32)

    for forward in np.arange(math.ceil(forward_min), forward_max + .01, 1.0):
        points = map_points(np.array(((forward, -lateral_limit),
                                      (forward, lateral_limit))))
        cv2.line(canvas, tuple(points[0]), tuple(points[1]), (220, 220, 220), 1)
        cv2.putText(canvas, "{:.0f}m".format(forward), (8, points[0, 1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (90, 90, 90), 1, cv2.LINE_AA)
    for lateral in np.arange(-math.floor(lateral_limit), lateral_limit + .01, 1.0):
        points = map_points(np.array(((forward_min, lateral), (forward_max, lateral))))
        cv2.line(canvas, tuple(points[0]), tuple(points[1]), (228, 228, 228), 1)
    if len(upper):
        for point in map_points(upper):
            cv2.circle(canvas, tuple(point), 2, (255, 0, 255), -1)
    if len(lower):
        for point in map_points(lower):
            cv2.circle(canvas, tuple(point), 2, (0, 140, 255), -1)
    origin = map_points(np.array(((0.0, 0.0),)))[0]
    tip = map_points(np.array(((0.35, 0.0),)))[0]
    cv2.arrowedLine(canvas, tuple(origin), tuple(tip), (0, 0, 220), 4,
                    cv2.LINE_AA, tipLength=.3)
    cv2.putText(canvas, "robot-relative BEV  t={:.1f}s".format(timestamp),
                (margin, 32), cv2.FONT_HERSHEY_SIMPLEX, .72, (20, 20, 20), 2,
                cv2.LINE_AA)
    cv2.putText(canvas, "magenta: upper z=0.254m   orange: lower z=0",
                (margin, 56), cv2.FONT_HERSHEY_SIMPLEX, .52, (60, 60, 60), 1,
                cv2.LINE_AA)
    return canvas


def draw_extraction(image, upper, lower, timestamp):
    overlay = image.copy()
    for point in upper:
        cv2.circle(overlay, tuple(np.rint(point).astype(int)), 1, (255, 255, 0), -1)
    for point in lower:
        cv2.circle(overlay, tuple(np.rint(point).astype(int)), 1, (0, 165, 255), -1)
    cv2.putText(overlay, "whole HSV boundary  t={:.1f}s".format(timestamp),
                (24, 34), cv2.FONT_HERSHEY_SIMPLEX, .68, (255, 255, 255), 2,
                cv2.LINE_AA)
    return overlay


def combine_views(overlay, bev):
    target_height = 720
    bev_width = round(bev.shape[1] * target_height / bev.shape[0])
    resized_bev = cv2.resize(bev, (bev_width, target_height), interpolation=cv2.INTER_AREA)
    if overlay.shape[0] != target_height:
        overlay_width = round(overlay.shape[1] * target_height / overlay.shape[0])
        overlay = cv2.resize(overlay, (overlay_width, target_height),
                             interpolation=cv2.INTER_AREA)
    return np.hstack((overlay, resized_bev))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--projective-fit", type=Path, required=True)
    parser.add_argument("--hsv", type=Path, required=True)
    parser.add_argument("--residual-warp", type=Path,
                        help="optional thin-plate residual warp generated by fit_rectified_residual_warp.py")
    parser.add_argument("--plumb-line-correction", type=Path,
                        help="optional radial/tangential correction report")
    parser.add_argument("--rectified-calibration", type=Path,
                        help="camera YAML required with --plumb-line-correction")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--sample-fps", type=float, default=1.0)
    parser.add_argument("--fence-height", type=float, default=0.254)
    parser.add_argument("--minimum-component-area", type=int, default=500)
    parser.add_argument("--minimum-blue-span", type=int, default=8)
    parser.add_argument("--forward-min", type=float, default=-0.5)
    parser.add_argument("--forward-max", type=float, default=5.0)
    parser.add_argument("--lateral-limit", type=float, default=4.0)
    args = parser.parse_args()

    fit = json.loads(args.projective_fit.read_text(encoding="utf-8"))
    matrix = np.asarray(fit["projection_matrix"], np.float64)
    ranges = load_blue_ranges(args.hsv)
    residual_warp, maximum_warp = load_residual_warp(args.residual_warp)
    radial_coefficients = radial_matrix = None
    if args.plumb_line_correction:
        if not args.rectified_calibration:
            parser.error("--rectified-calibration is required with --plumb-line-correction")
        correction = json.loads(args.plumb_line_correction.read_text(encoding="utf-8"))
        values = correction["coefficients"]
        radial_coefficients = np.array((values["k1"], values["k2"],
                                        values["p1"], values["p2"]), np.float64)
        storage = cv2.FileStorage(str(args.rectified_calibration), cv2.FILE_STORAGE_READ)
        radial_matrix = storage.getNode("rectified_K").mat()
        storage.release()
    output_dir = args.output_dir or args.video.with_name(args.video.stem + "-bev")
    raw_dir, bev_dir = output_dir / "edges", output_dir / "bev"
    raw_dir.mkdir(parents=True, exist_ok=True)
    bev_dir.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("cannot open {}".format(args.video))
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if source_fps <= 0 or args.sample_fps <= 0 or args.sample_fps > source_fps:
        raise ValueError("sample FPS must be positive and no greater than source FPS")
    interval = source_fps / args.sample_fps
    next_sample = 0.0
    frame_index = sample_index = 0
    video_writer = None
    total_upper = total_lower = 0
    cloud_distances = []
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index + 1e-6 < next_sample:
                frame_index += 1
                continue
            timestamp = frame_index / source_fps
            mask = fence_mask(frame, ranges, args.minimum_component_area)
            upper_pixels, lower_pixels = boundary_pixels(mask, args.minimum_blue_span)
            corrected_upper = correct_radial_pixels(
                upper_pixels, radial_coefficients, radial_matrix)
            corrected_lower = correct_radial_pixels(
                lower_pixels, radial_coefficients, radial_matrix)
            upper = filter_robot_points(
                image_to_robot(matrix, correct_pixels(
                    corrected_upper, residual_warp, maximum_warp), args.fence_height),
                args.forward_min, args.forward_max, args.lateral_limit)
            lower = filter_robot_points(
                image_to_robot(matrix, correct_pixels(
                    corrected_lower, residual_warp, maximum_warp), 0.0),
                args.forward_min, args.forward_max, args.lateral_limit)
            cloud_distances.extend(symmetric_cloud_distance(upper, lower))
            overlay = draw_extraction(frame, upper_pixels, lower_pixels, timestamp)
            bev = draw_bev(upper, lower, timestamp, args.forward_min,
                           args.forward_max, args.lateral_limit)
            combined = combine_views(overlay, bev)
            if video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                video_writer = cv2.VideoWriter(
                    str(output_dir / "edge-pointcloud-1fps.avi"), fourcc,
                    args.sample_fps, (combined.shape[1], combined.shape[0]))
            name = "frame-{:04d}".format(sample_index)
            cv2.imwrite(str(raw_dir / (name + ".jpg")), overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
            cv2.imwrite(str(bev_dir / (name + ".png")), bev)
            video_writer.write(combined)
            total_upper += len(upper)
            total_lower += len(lower)
            sample_index += 1
            next_sample += interval
            frame_index += 1
    finally:
        capture.release()
        if video_writer is not None:
            video_writer.release()

    cloud_distances = np.asarray(cloud_distances)
    summary = {
        "source": str(args.video), "source_fps": source_fps,
        "sample_fps": args.sample_fps, "sample_count": sample_index,
        "upper_point_count": total_upper, "lower_point_count": total_lower,
        "residual_warp": str(args.residual_warp) if args.residual_warp else None,
        "plumb_line_correction": (str(args.plumb_line_correction)
                                  if args.plumb_line_correction else None),
        "symmetric_upper_lower_nearest_distance_m": {
            "median": float(np.median(cloud_distances)),
            "p90": float(np.percentile(cloud_distances, 90)),
            "rmse": float(np.sqrt(np.mean(np.square(cloud_distances)))),
        },
        "coordinate_frame": "robot: +x forward, +y left",
        "limits_m": {"forward": [args.forward_min, args.forward_max],
                     "left_right": [-args.lateral_limit, args.lateral_limit]},
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                               encoding="utf-8")
    print("sampled {} frames; projected {} upper and {} lower points".format(
        sample_index, total_upper, total_lower))
    print("wrote {}".format(output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
