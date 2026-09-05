#!/usr/bin/env python3
"""Test and fit small residual distortion from straight arena fence edges."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

import regress_arena_camera as camera


def boundary_pixels(mask):
    upper, lower = [], []
    for x in range(mask.shape[1]):
        rows = np.flatnonzero(mask[:, x])
        if len(rows) < 2 or rows[-1] - rows[0] < 8:
            continue
        upper.append((float(x), float(rows[0])))
        lower.append((float(x), float(rows[-1])))
    return np.asarray(upper), np.asarray(lower)


def segment_points(contour, endpoints, endpoint_margin, maximum_distance):
    start, end = np.asarray(endpoints, np.float64)
    delta = end - start
    length_squared = float(delta @ delta)
    if length_squared < 100.0:
        return np.empty((0, 2), np.float64)
    relative = contour - start
    fraction = relative @ delta / length_squared
    distance = np.abs(relative[:, 0] * delta[1] - relative[:, 1] * delta[0]) / math.sqrt(
        length_squared)
    selected = ((fraction >= endpoint_margin) & (fraction <= 1.0 - endpoint_margin) &
                (distance <= maximum_distance))
    return contour[selected]


def fit_line(points):
    centre = np.mean(points, axis=0)
    _, _, right = np.linalg.svd(points - centre, full_matrices=False)
    direction = right[0]
    normal = np.array((-direction[1], direction[0]))
    if normal[1] < 0:
        normal = -normal
    rho = float(normal @ centre)
    return math.atan2(normal[1], normal[0]), rho


def line_errors(points):
    theta, rho = fit_line(points)
    normal = np.array((math.cos(theta), math.sin(theta)))
    return points @ normal - rho


def reject_outliers(points):
    if len(points) < 8:
        return points
    errors = line_errors(points)
    median = np.median(errors)
    mad = np.median(np.abs(errors - median))
    threshold = max(2.0, 4.0 * 1.4826 * mad)
    return points[np.abs(errors - median) <= threshold]


def correct_points(points, coefficients, matrix):
    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    x = (points[:, 0] - cx) / fx
    y = (points[:, 1] - cy) / fy
    r2 = x * x + y * y
    k1, k2, p1, p2 = coefficients
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    corrected_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    corrected_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return np.column_stack((fx * corrected_x + cx, fy * corrected_y + cy))


def residual(parameters, segments, matrix, coefficient_sigma):
    coefficients = parameters[:4]
    line_parameters = parameters[4:].reshape(-1, 2)
    values = []
    for segment, (theta, rho) in zip(segments, line_parameters):
        points = correct_points(segment["points"], coefficients, matrix)
        normal = np.array((math.cos(theta), math.sin(theta)))
        values.extend(points @ normal - rho)
    values.extend(coefficients / coefficient_sigma)
    return np.asarray(values)


def fit_correction(segments, matrix, coefficient_sigma):
    lines = np.asarray([fit_line(item["points"]) for item in segments]).ravel()
    initial = np.concatenate((np.zeros(4), lines))
    lower = np.concatenate(((-.03, -.02, -.01, -.01),
                            np.tile((-np.inf, -np.inf), len(segments))))
    upper = np.concatenate(((.03, .02, .01, .01),
                            np.tile((np.inf, np.inf), len(segments))))
    result = least_squares(
        residual, initial, bounds=(lower, upper),
        args=(segments, matrix, coefficient_sigma), loss="linear",
        x_scale=np.concatenate(((.005, .005, .002, .002),
                                np.tile((.05, 100.0), len(segments)))),
        max_nfev=1000)
    return result.x[:4]


def straightness(segments, coefficients, matrix):
    errors = []
    per_segment = []
    for segment in segments:
        points = correct_points(segment["points"], coefficients, matrix)
        values = line_errors(points)
        errors.extend(values)
        per_segment.append(float(np.sqrt(np.mean(np.square(values)))))
    errors = np.asarray(errors)
    return {
        "rms_px": float(np.sqrt(np.mean(np.square(errors)))),
        "p90_abs_px": float(np.percentile(np.abs(errors), 90)),
        "max_abs_px": float(np.max(np.abs(errors))),
        "median_segment_rms_px": float(np.median(per_segment)),
    }


def render_pose(path, image, segments, coefficients, matrix):
    view = image.copy()
    colours = {"upper": (255, 255, 0), "lower": (0, 165, 255)}
    for segment in segments:
        original = segment["points"]
        corrected = correct_points(original, coefficients, matrix)
        colour = colours[segment["edge"]]
        for point in original[::8]:
            cv2.circle(view, tuple(np.rint(point).astype(int)), 2, colour, -1)
        for source, target in zip(original[::24], corrected[::24]):
            exaggerated = source + 10.0 * (target - source)
            cv2.arrowedLine(view, tuple(np.rint(source).astype(int)),
                            tuple(np.rint(exaggerated).astype(int)),
                            (0, 0, 255), 1, cv2.LINE_AA, tipLength=.25)
    cv2.rectangle(view, (0, 0), (view.shape[1], 60), (20, 20, 20), -1)
    cv2.putText(view, "HSV plumb-line samples; red arrows show residual correction at 10x",
                (20, 36), cv2.FONT_HERSHEY_SIMPLEX, .65,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), view, [cv2.IMWRITE_JPEG_QUALITY, 94])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--tags", type=Path, required=True)
    parser.add_argument("--hsv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--endpoint-margin", type=float, default=.08)
    parser.add_argument("--maximum-tag-distance", type=float, default=15.0)
    parser.add_argument("--minimum-segment-points", type=int, default=24)
    args = parser.parse_args()

    session = json.loads((args.session / "poses.json").read_text(encoding="utf-8"))
    entries = [entry for entry in json.loads(args.tags.read_text(encoding="utf-8"))["poses"]
               if entry.get("reviewed")]
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    _, _, rectified_matrix, width, height = camera.load_calibration(calibration_path)
    ranges = camera.load_blue_ranges(args.hsv)
    output_dir = args.output_dir or args.session / "plumb-line-test"
    overlay_dir = output_dir / "overlays"
    overlay_dir.mkdir(parents=True, exist_ok=True)

    segments = []
    images = {}
    for entry in entries:
        image = cv2.imread(str(args.session / entry["rectified_image"]), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise ValueError("cannot load pose {} rectified image".format(entry["id"]))
        images[entry["id"]] = image
        mask = camera.blue_mask(image, ranges, 9)
        upper, lower = boundary_pixels(mask)
        for edge_name, contour in (("upper", upper), ("lower", lower)):
            vertices = entry.get(edge_name, [])
            for index, endpoints in enumerate(zip(vertices[:-1], vertices[1:])):
                points = segment_points(contour, endpoints, args.endpoint_margin,
                                        args.maximum_tag_distance)
                points = reject_outliers(points)
                if len(points) < args.minimum_segment_points:
                    continue
                segments.append({"pose_id": entry["id"], "edge": edge_name,
                                 "index": index, "points": points})
    if len(segments) < 12:
        raise ValueError("only {} usable straight spans".format(len(segments)))

    # Straight-line distance weakly observes motion along each line. Keep
    # tangential terms near zero unless the data provides overwhelming evidence.
    coefficient_sigma = np.array((.01, .01, .0001, .0001))
    coefficients = fit_correction(segments, rectified_matrix, coefficient_sigma)
    baseline = straightness(segments, np.zeros(4), rectified_matrix)
    fitted = straightness(segments, coefficients, rectified_matrix)

    heldout_rows = []
    baseline_heldout, corrected_heldout = [], []
    for pose_id in sorted({item["pose_id"] for item in segments}):
        train = [item for item in segments if item["pose_id"] != pose_id]
        test = [item for item in segments if item["pose_id"] == pose_id]
        if not train or not test:
            continue
        fold_coefficients = fit_correction(train, rectified_matrix, coefficient_sigma)
        before = straightness(test, np.zeros(4), rectified_matrix)
        after = straightness(test, fold_coefficients, rectified_matrix)
        baseline_heldout.append(before["rms_px"])
        corrected_heldout.append(after["rms_px"])
        heldout_rows.append({"pose_id": pose_id, "baseline_rms_px": before["rms_px"],
                             "corrected_rms_px": after["rms_px"]})

    for pose_id, image in images.items():
        pose_segments = [item for item in segments if item["pose_id"] == pose_id]
        render_pose(overlay_dir / "pose-{:02d}.jpg".format(pose_id), image,
                    pose_segments, coefficients, rectified_matrix)
    report = {
        "model": "residual_radial_tangential_after_fisheye_rectification",
        "coefficients": {"k1": coefficients[0], "k2": coefficients[1],
                         "p1": coefficients[2], "p2": coefficients[3]},
        "pose_count": len({item["pose_id"] for item in segments}),
        "segment_count": len(segments),
        "point_count": sum(len(item["points"]) for item in segments),
        "all_data": {"baseline": baseline, "corrected": fitted},
        "leave_one_pose_out": {
            "baseline_mean_rms_px": float(np.mean(baseline_heldout)),
            "corrected_mean_rms_px": float(np.mean(corrected_heldout)),
            "poses": heldout_rows,
        },
    }
    (output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n",
                                             encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("wrote {}".format(output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
