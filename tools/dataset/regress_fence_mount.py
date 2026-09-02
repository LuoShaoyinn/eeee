#!/usr/bin/env python3
"""Regress camera height, pitch, and roll from fixed-height fence edges.

The physical fence planes are fixed at z=0 and z=0.254 m.  This tool assumes
camera yaw is zero; yaw is irrelevant to the upper/lower curve-overlap loss.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import minimize

from estimate_fence_pose import CAMERA_MATRIX, FENCE_HEIGHT_M
from fit_fence_edges import column_edges, retain_fence_components
from hsv_thresholds import load_thresholds, make_mask


def rotation_car_from_camera(pitch_deg: float, roll_deg: float) -> np.ndarray:
    pitch = math.radians(pitch_deg)
    sine, cosine = math.sin(pitch), math.cos(pitch)
    pitch_rotation = np.array([[0.0, -sine, cosine], [-1.0, 0.0, 0.0],
                               [0.0, -cosine, -sine]], dtype=float)
    roll = math.radians(roll_deg)
    optical_roll = np.array([[math.cos(roll), -math.sin(roll), 0.0],
                             [math.sin(roll), math.cos(roll), 0.0],
                             [0.0, 0.0, 1.0]], dtype=float)
    return pitch_rotation @ optical_roll


def project_to_plane(pixels: np.ndarray, camera_height_m: float, pitch_deg: float,
                     roll_deg: float, plane_z_m: float) -> np.ndarray:
    rays = np.column_stack(((pixels[:, 0] - CAMERA_MATRIX[0, 2]) / CAMERA_MATRIX[0, 0],
                            (pixels[:, 1] - CAMERA_MATRIX[1, 2]) / CAMERA_MATRIX[1, 1],
                            np.ones(len(pixels))))
    directions = rays @ rotation_car_from_camera(pitch_deg, roll_deg).T
    scale = (plane_z_m - camera_height_m) / directions[:, 2]
    points = directions[:, :2] * scale[:, None]
    valid = np.isfinite(points).all(axis=1) & (scale > 0)
    return points[valid]


def symmetric_chamfer(first: np.ndarray, second: np.ndarray) -> float:
    if len(first) < 20 or len(second) < 20:
        return .25
    # Fixed subsampling bounds the quadratic reference implementation.
    first = first[::max(1, len(first) // 100)]
    second = second[::max(1, len(second) // 100)]
    distance = np.linalg.norm(first[:, None, :] - second[None, :, :], axis=2)
    return float(.5 * (np.median(distance.min(axis=1)) + np.median(distance.min(axis=0))))


def collect_pairs(paths, thresholds, x_min, x_max):
    pairs = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        blue = retain_fence_components(cv2.morphologyEx(
            make_mask(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), thresholds["blue_fence"]),
            cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
        xs, upper_y, lower_y, _, _ = column_edges(blue, min_pixels=8)
        keep = (xs >= x_min) & (xs <= x_max)
        if np.count_nonzero(keep) >= 40:
            pairs.append((path.name, np.column_stack((xs[keep], lower_y[keep])),
                          np.column_stack((xs[keep], upper_y[keep]))))
    return pairs


def scores(parameters, pairs):
    height, pitch, roll = parameters
    return np.asarray([symmetric_chamfer(
        project_to_plane(lower, height, pitch, roll, 0.0),
        project_to_plane(upper, height, pitch, roll, FENCE_HEIGHT_M))
        for _, lower, upper in pairs])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--glob", default="capture-20260831-100146-0001_*.jpg")
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--x-min", type=int, default=20)
    parser.add_argument("--x-max", type=int, default=250,
                        help="unoccluded left image range used for this regression")
    parser.add_argument("--keep-fraction", type=float, default=.65,
                        help="best per-frame curve matches retained for the next robust fit")
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--output", type=Path, default=Path("/tmp/fence-mount-regression.json"))
    args = parser.parse_args()
    thresholds = load_thresholds(args.thresholds)
    pairs = collect_pairs(sorted(args.images.glob(args.glob)), thresholds, args.x_min, args.x_max)
    if len(pairs) < 3:
        parser.error("need at least three usable images")

    if not 0 < args.keep_fraction <= 1 or args.iterations < 1:
        parser.error("keep fraction must be in (0, 1] and iterations must be positive")

    active = np.arange(len(pairs))

    def objective(parameters, indices):
        values = scores(parameters, [pairs[index] for index in indices])
        # Cap individual bad/occluded frames so they cannot dominate the fit.
        return float(np.mean(np.minimum(values, .10)))

    nominal = np.array([.1311825723, 30.1132498, 0.0])
    result = None
    seed = np.array([.136, 29.9, 0.0])
    for _ in range(args.iterations):
        result = minimize(lambda parameters: objective(parameters, active), x0=seed, method="Powell",
                          bounds=((.110, .160), (27.0, 33.0), (-4.0, 4.0)),
                          options={"xtol": 1e-5, "ftol": 1e-6, "maxiter": 300})
        seed = result.x
        all_values = scores(seed, pairs)
        keep_count = max(3, round(len(pairs) * args.keep_fraction))
        active = np.argsort(all_values)[:keep_count]
    fitted = result.x
    values = scores(fitted, pairs)
    payload = {
        "assumed_yaw_deg": 0.0,
        "physical_fence_height_m": FENCE_HEIGHT_M,
        "fit": {"camera_height_m": float(fitted[0]), "pitch_down_deg": float(fitted[1]),
                "roll_deg": float(fitted[2]), "objective_m": float(result.fun),
                "success": bool(result.success), "message": result.message,
                "retained_frame_count": int(len(active))},
        "nominal": {"camera_height_m": float(nominal[0]), "pitch_down_deg": float(nominal[1]),
                    "roll_deg": 0.0, "median_offset_m": float(np.median(scores(nominal, pairs)))} ,
        "frames": [{"name": name, "offset_m": float(value)} for (name, _, _), value in zip(pairs, values)],
    }
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print("frames={} yaw_assumed_deg=0".format(len(pairs)))
    print("fit: height_m={:.5f} pitch_deg={:.4f} roll_deg={:.4f}".format(*fitted))
    print("median_offset_m: nominal={:.4f} fitted={:.4f}".format(
        payload["nominal"]["median_offset_m"], float(np.median(values))))
    print("wrote {}".format(args.output))


if __name__ == "__main__":
    main()
