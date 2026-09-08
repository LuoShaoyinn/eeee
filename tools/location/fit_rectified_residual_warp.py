#!/usr/bin/env python3
"""Fit a smooth residual pixel warp on top of a projective camera model."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.interpolate import RBFInterpolator

import regress_arena_camera as camera
import regress_arena_projective as projective


def corrected_poses(tags, fit):
    corrections = {item["id"]: item["correction"] for item in fit["poses"]}
    poses = []
    for entry in tags:
        correction = corrections[entry["id"]]
        poses.append((entry["x_m"] + correction["x_m"],
                      entry["y_m"] + correction["y_m"],
                      math.radians(entry["yaw_deg"] + correction["yaw_deg"])))
    return np.asarray(poses, np.float64)


def collect_controls(tags, observations, poses, matrix, arena_x, arena_y):
    pixels, targets, pose_ids = [], [], []
    for entry, observation, pose in zip(tags, observations, poses):
        for corner in camera.tagged_corners(observation):
            world_xy = projective.arena_corner_for_walls(
                corner["walls"], arena_x, arena_y)
            target = projective.project_point(
                matrix, projective.robot_point(world_xy, corner["edge_height"], pose))
            pixels.append(corner["pixel"])
            targets.append(target)
            pose_ids.append(entry["id"])
    return (np.asarray(pixels, np.float64), np.asarray(targets, np.float64),
            np.asarray(pose_ids, np.int32))


def capped_displacement(interpolator, pixels, maximum):
    displacement = interpolator(pixels)
    lengths = np.linalg.norm(displacement, axis=1)
    scale = np.minimum(1.0, maximum / np.maximum(lengths, 1e-9))
    return displacement * scale[:, None]


def cross_validate(pixels, displacement, pose_ids, smoothing_values, maximum):
    rows = []
    for smoothing in smoothing_values:
        errors = []
        for pose_id in np.unique(pose_ids):
            train = pose_ids != pose_id
            test = ~train
            if np.count_nonzero(train) < 6:
                continue
            interpolator = RBFInterpolator(
                pixels[train], displacement[train], kernel="thin_plate_spline",
                smoothing=smoothing, degree=1)
            predicted = capped_displacement(interpolator, pixels[test], maximum)
            errors.extend(np.linalg.norm(predicted - displacement[test], axis=1))
        errors = np.asarray(errors)
        rows.append({
            "smoothing": smoothing,
            "median_px": float(np.median(errors)),
            "rmse_px": float(np.sqrt(np.mean(np.square(errors)))),
            "p90_px": float(np.percentile(errors, 90)),
        })
    return rows


def render_field(path, interpolator, width, height, maximum):
    step = 40
    xs, ys = np.meshgrid(np.arange(0, width, step), np.arange(0, height, step))
    points = np.column_stack((xs.ravel(), ys.ravel())).astype(np.float64)
    displacement = capped_displacement(interpolator, points, maximum)
    canvas = np.full((height, width, 3), 248, np.uint8)
    for point, delta in zip(points, displacement):
        start = tuple(np.rint(point).astype(int))
        end = tuple(np.rint(point + 5.0 * delta).astype(int))
        cv2.arrowedLine(canvas, start, end, (180, 50, 20), 1, cv2.LINE_AA,
                        tipLength=.25)
    cv2.putText(canvas, "residual warp vectors (5x display scale)", (24, 34),
                cv2.FONT_HERSHEY_SIMPLEX, .68, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), canvas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--tags", type=Path, required=True)
    parser.add_argument("--projective-fit", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--maximum-displacement", type=float, default=25.0)
    parser.add_argument("--smoothing", type=float,
                        help="override leave-one-pose-out smoothing selection")
    args = parser.parse_args()

    session = json.loads((args.session / "poses.json").read_text(encoding="utf-8"))
    tag_file = json.loads(args.tags.read_text(encoding="utf-8"))
    tags = [entry for entry in tag_file.get("poses", []) if entry.get("reviewed")]
    fit = json.loads(args.projective_fit.read_text(encoding="utf-8"))
    matrix = np.asarray(fit["projection_matrix"], np.float64)
    arena_x = float(session["arena"]["x_length_m"])
    arena_y = float(session["arena"]["y_length_m"])
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    _, _, rectified_matrix, width, height = camera.load_calibration(calibration_path)

    observations = []
    measured_poses = []
    for entry in tags:
        segments = (camera.tagged_segments(entry.get("upper", []), .254) +
                    camera.tagged_segments(entry.get("lower", []), 0.0))
        observations.append({"id": entry["id"], "segments": segments})
        measured_poses.append((entry["x_m"], entry["y_m"],
                               math.radians(entry["yaw_deg"])))
    measured_poses = np.asarray(measured_poses, np.float64)
    initial_mount = np.array((-.04, 0.0, .145, .5, 30.4, 0.0))
    camera.assign_walls(observations, measured_poses, initial_mount,
                        rectified_matrix, arena_x, arena_y)
    poses = corrected_poses(tags, fit)
    pixels, targets, pose_ids = collect_controls(
        tags, observations, poses, matrix, arena_x, arena_y)
    displacement = targets - pixels

    smoothing_values = [0.0, 1.0, 10.0, 100.0, 1000.0,
                        10000.0, 100000.0, 1000000.0]
    validation = cross_validate(pixels, displacement, pose_ids,
                                smoothing_values, args.maximum_displacement)
    zero_warp_rmse = float(np.sqrt(np.mean(np.square(
        np.linalg.norm(displacement, axis=1)))))
    smoothing = (args.smoothing if args.smoothing is not None
                 else min(validation, key=lambda row: row["rmse_px"])["smoothing"])
    selected_validation = next(row for row in validation
                               if row["smoothing"] == smoothing)
    interpolator = RBFInterpolator(
        pixels, displacement, kernel="thin_plate_spline",
        smoothing=smoothing, degree=1)
    fitted_displacement = capped_displacement(
        interpolator, pixels, args.maximum_displacement)
    residual = np.linalg.norm(fitted_displacement - displacement, axis=1)

    output = args.output or args.session / "regression-projective" / "residual-warp.npz"
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, control_pixels=pixels,
                        control_displacements=displacement,
                        smoothing=np.array(smoothing),
                        maximum_displacement=np.array(args.maximum_displacement),
                        image_size=np.array((width, height), np.int32))
    report = {
        "model": "thin_plate_spline_pixel_residual",
        "control_count": len(pixels), "pose_count": len(np.unique(pose_ids)),
        "selected_smoothing": smoothing,
        "accepted": selected_validation["rmse_px"] < zero_warp_rmse,
        "zero_warp_validation_rmse_px": zero_warp_rmse,
        "selected_warp_validation_rmse_px": selected_validation["rmse_px"],
        "maximum_displacement_px": args.maximum_displacement,
        "control_displacement_px": {
            "median": float(np.median(np.linalg.norm(displacement, axis=1))),
            "p90": float(np.percentile(np.linalg.norm(displacement, axis=1), 90)),
            "max": float(np.max(np.linalg.norm(displacement, axis=1))),
        },
        "training_residual_px": {
            "median": float(np.median(residual)),
            "rmse": float(np.sqrt(np.mean(np.square(residual)))),
            "max": float(np.max(residual)),
        },
        "leave_one_pose_out": validation,
    }
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n",
                                            encoding="utf-8")
    render_field(output.with_name(output.stem + "-field.png"), interpolator,
                 width, height, args.maximum_displacement)
    print(json.dumps(report, indent=2))
    print("wrote {}".format(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
