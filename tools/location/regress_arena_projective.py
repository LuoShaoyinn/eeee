#!/usr/bin/env python3
"""Fit one full 3x4 robot-frame projective camera matrix from arena corners."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

import regress_arena_camera as camera


def arena_corner_for_walls(walls, arena_x, arena_y):
    corners = np.array(((0.0, 0.0), (arena_x, 0.0),
                        (arena_x, arena_y), (0.0, arena_y)), np.float64)
    first = {walls[0], (walls[0] + 1) % 4}
    second = {walls[1], (walls[1] + 1) % 4}
    shared = first & second
    if len(shared) != 1:
        raise ValueError("walls {} do not share one arena corner".format(walls))
    return corners[shared.pop()]


def robot_point(world_xy, height, pose):
    delta = np.asarray(world_xy, np.float64) - pose[:2]
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return np.array((cosine * delta[0] + sine * delta[1],
                     -sine * delta[0] + cosine * delta[1],
                     height, 1.0), np.float64)


def physical_matrix(mount, camera_matrix):
    centre, rotation_world_from_camera = camera.camera_frame(mount, 0.0)
    rotation_camera_from_robot = rotation_world_from_camera.T
    extrinsic = np.column_stack((rotation_camera_from_robot,
                                 -rotation_camera_from_robot @ centre))
    return camera_matrix @ extrinsic


def matrix_codec(initial):
    gauge_index = int(np.argmax(np.abs(initial)))
    normalized = initial / initial.flat[gauge_index]
    variable_indices = [index for index in range(12) if index != gauge_index]

    def encode(matrix):
        normalized_matrix = matrix / matrix.flat[gauge_index]
        return normalized_matrix.ravel()[variable_indices]

    def decode(parameters):
        flat = np.empty(12, np.float64)
        flat[gauge_index] = 1.0
        flat[variable_indices] = parameters
        return flat.reshape(3, 4)

    return encode(normalized), decode, gauge_index


def project_point(matrix, point):
    homogeneous = matrix @ point
    if abs(homogeneous[2]) < 1e-9:
        return np.array((1e6, 1e6))
    return homogeneous[:2] / homogeneous[2]


def corner_residual(matrix_parameters, decode, observations, poses,
                    arena_x, arena_y, pixel_sigma):
    matrix = decode(matrix_parameters)
    residuals = []
    for observation, pose in zip(observations, poses):
        for corner in camera.tagged_corners(observation):
            world_xy = arena_corner_for_walls(corner["walls"], arena_x, arena_y)
            point = robot_point(world_xy, corner["edge_height"], pose)
            residuals.extend((project_point(matrix, point) - corner["pixel"]) / pixel_sigma)
    return np.asarray(residuals)


def joint_residual(parameters, decode, observations, measured_poses,
                   arena_x, arena_y, pixel_sigma, position_sigma, yaw_sigma):
    matrix_parameters = parameters[:11]
    corrections = parameters[11:].reshape(-1, 3)
    poses = measured_poses + corrections
    image = corner_residual(matrix_parameters, decode, observations, poses,
                            arena_x, arena_y, pixel_sigma)
    priors = corrections.copy()
    priors[:, :2] /= position_sigma
    priors[:, 2] /= yaw_sigma
    return np.concatenate((image, priors.ravel()))


def project_world(matrix, points, pose):
    projected = []
    visible = []
    for point in points:
        robot = robot_point(point[:2], point[2], pose)
        homogeneous = matrix @ robot
        visible.append(homogeneous[2] > 0.0)
        projected.append(homogeneous[:2] / homogeneous[2])
    return np.asarray(projected), np.asarray(visible)


def image_to_world(matrix, pixel, height, pose):
    homography = np.column_stack((matrix[:, 0], matrix[:, 1],
                                  height * matrix[:, 2] + matrix[:, 3]))
    robot = np.linalg.solve(homography, np.array((pixel[0], pixel[1], 1.0)))
    robot /= robot[2]
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    world = pose[:2] + np.array((cosine * robot[0] - sine * robot[1],
                                sine * robot[0] + cosine * robot[1]))
    return world


def collect_corner_errors(matrix, observations, poses, arena_x, arena_y):
    errors = []
    for observation, pose in zip(observations, poses):
        for corner in camera.tagged_corners(observation):
            world_xy = arena_corner_for_walls(corner["walls"], arena_x, arena_y)
            predicted = project_point(matrix, robot_point(
                world_xy, corner["edge_height"], pose))
            errors.append(float(np.linalg.norm(predicted - corner["pixel"])))
    return np.asarray(errors)


def error_summary(errors):
    return {
        "median_error_px": float(np.median(errors)),
        "p90_error_px": float(np.percentile(errors, 90)),
        "rmse_error_px": float(np.sqrt(np.mean(np.square(errors)))),
        "max_error_px": float(np.max(errors)),
    }


def render(output_dir, observations, poses, matrix, arena_x, arena_y,
           fence_height, pixel_sigma):
    overlay_dir = output_dir / "overlays"
    bev_dir = output_dir / "bev"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    bev_dir.mkdir(parents=True, exist_ok=True)
    edges = camera.arena_edges(arena_x, arena_y, fence_height)
    scale, margin_m = 280, 0.5
    border = round(margin_m * scale)
    canvas_size = (round(arena_x * scale) + 2 * border,
                   round(arena_y * scale) + 2 * border)
    angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
    offsets = pixel_sigma * np.column_stack((np.cos(angles), np.sin(angles)))

    for observation, pose in zip(observations, poses):
        overlay = observation["image"].copy()
        for index, edge in enumerate(edges):
            projected, visible = project_world(matrix, edge, pose)
            camera.draw_polyline(overlay, projected, visible,
                                 (255, 0, 255) if index % 2 else (0, 255, 0))
        for corner in camera.tagged_corners(observation):
            world_xy = arena_corner_for_walls(corner["walls"], arena_x, arena_y)
            predicted = project_point(
                matrix, robot_point(world_xy, corner["edge_height"], pose))
            observed = tuple(np.rint(corner["pixel"]).astype(int))
            fitted = tuple(np.rint(predicted).astype(int))
            error = np.linalg.norm(predicted - corner["pixel"])
            colour = (255, 255, 0) if corner["edge_height"] else (0, 165, 255)
            cv2.circle(overlay, observed, round(pixel_sigma), colour, 2, cv2.LINE_AA)
            cv2.line(overlay, observed, fitted, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.drawMarker(overlay, fitted, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
            cv2.putText(overlay, "{:.1f}px".format(error),
                        (observed[0] + 7, observed[1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, "full 3x4 P: circles observed; crosses fitted",
                    (24, 34), cv2.FONT_HERSHEY_SIMPLEX, .62,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(overlay_dir / "pose-{:02d}.jpg".format(observation["id"])),
                    overlay, [cv2.IMWRITE_JPEG_QUALITY, 94])

        canvas = np.full((canvas_size[1], canvas_size[0], 3), 245, np.uint8)

        def map_point(point):
            return (round(border + point[0] * scale),
                    round(canvas_size[1] - border - point[1] * scale))

        cv2.rectangle(canvas, map_point((0.0, arena_y)), map_point((arena_x, 0.0)),
                      (20, 20, 20), 3)
        uncertainty = canvas.copy()
        for corner in camera.tagged_corners(observation):
            colour = ((255, 190, 255) if corner["edge_height"]
                      else (120, 205, 255))
            points = []
            for pixel in corner["pixel"] + offsets:
                try:
                    point = image_to_world(matrix, pixel, corner["edge_height"], pose)
                except np.linalg.LinAlgError:
                    continue
                point = np.clip(point, (-margin_m, -margin_m),
                                (arena_x + margin_m, arena_y + margin_m))
                points.append(map_point(point))
            if len(points) >= 3:
                polygon = cv2.convexHull(np.asarray(points, np.int32))
                cv2.fillConvexPoly(uncertainty, polygon, colour, cv2.LINE_AA)
            centre = image_to_world(matrix, corner["pixel"], corner["edge_height"], pose)
            cv2.circle(canvas, map_point(centre), 4, colour, -1, cv2.LINE_AA)
        cv2.addWeighted(uncertainty, .30, canvas, .70, 0.0, canvas)
        origin = map_point(pose[:2])
        direction = np.array((math.cos(pose[2]), math.sin(pose[2])))
        cv2.arrowedLine(canvas, origin, map_point(pose[:2] + .2 * direction),
                        (0, 0, 220), 3, cv2.LINE_AA, tipLength=.3)
        cv2.putText(canvas, "pose {:02d}: full 3x4 P; +/-{:.0f}px".format(
                    observation["id"], pixel_sigma), (15, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .60, (30, 30, 30), 2, cv2.LINE_AA)
        cv2.imwrite(str(bev_dir / "pose-{:02d}.png".format(observation["id"])), canvas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--tags", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fence-height", type=float, default=0.254)
    parser.add_argument("--pixel-sigma", type=float, default=3.0)
    parser.add_argument("--position-sigma", type=float, default=0.02)
    parser.add_argument("--yaw-sigma-deg", type=float, default=1.0)
    parser.add_argument("--position-correction-limit", type=float, default=0.02)
    parser.add_argument("--yaw-correction-limit-deg", type=float, default=1.0)
    args = parser.parse_args()

    session = json.loads((args.session / "poses.json").read_text(encoding="utf-8"))
    tags = json.loads(args.tags.read_text(encoding="utf-8"))
    tagged = [entry for entry in tags.get("poses", []) if entry.get("reviewed")]
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    _, _, rectified_matrix, width, height = camera.load_calibration(calibration_path)
    arena_x = float(session["arena"]["x_length_m"])
    arena_y = float(session["arena"]["y_length_m"])
    output_dir = args.output_dir or args.session / "regression-projective"
    output_dir.mkdir(parents=True, exist_ok=True)

    observations, measured_poses = [], []
    for entry in tagged:
        image = cv2.imread(str(args.session / entry["rectified_image"]), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise ValueError("cannot load pose {} rectified image".format(entry["id"]))
        segments = (camera.tagged_segments(entry.get("upper", []), args.fence_height) +
                    camera.tagged_segments(entry.get("lower", []), 0.0))
        observations.append({"id": entry["id"], "image": image, "segments": segments})
        measured_poses.append((entry["x_m"], entry["y_m"],
                               math.radians(entry["yaw_deg"])))
    measured_poses = np.asarray(measured_poses, np.float64)

    initial_mount = np.array((-.04, 0.0, .145, .5, 30.4, 0.0))
    assignments = camera.assign_walls(observations, measured_poses, initial_mount,
                                      rectified_matrix, arena_x, arena_y)
    for observation, walls in zip(observations, assignments):
        print("pose {:02d}: wall assignments {}".format(observation["id"], walls))

    initial_matrix = physical_matrix(initial_mount, rectified_matrix)
    initial_parameters, decode, gauge_index = matrix_codec(initial_matrix)
    fixed_fit = least_squares(
        corner_residual, initial_parameters,
        args=(decode, observations, measured_poses, arena_x, arena_y, args.pixel_sigma),
        loss="linear", x_scale="jac", max_nfev=2000, verbose=1)
    fixed_matrix = decode(fixed_fit.x)
    fixed_errors = collect_corner_errors(fixed_matrix, observations, measured_poses,
                                         arena_x, arena_y)

    position_limit = args.position_correction_limit
    yaw_limit = math.radians(args.yaw_correction_limit_deg)
    lower = np.tile((-position_limit, -position_limit, -yaw_limit), len(observations))
    upper = np.tile((position_limit, position_limit, yaw_limit), len(observations))
    joint_initial = np.concatenate((fixed_fit.x, np.zeros(3 * len(observations))))
    joint_fit = least_squares(
        joint_residual, joint_initial,
        bounds=(np.concatenate((np.full(11, -np.inf), lower)),
                np.concatenate((np.full(11, np.inf), upper))),
        args=(decode, observations, measured_poses, arena_x, arena_y, args.pixel_sigma,
              args.position_sigma, math.radians(args.yaw_sigma_deg)),
        loss="linear", x_scale="jac", max_nfev=3000, verbose=1)
    matrix = decode(joint_fit.x[:11])
    corrections = joint_fit.x[11:].reshape(-1, 3)
    fitted_poses = measured_poses + corrections

    reports, errors = [], []
    for observation, measured, fitted, correction in zip(
            observations, measured_poses, fitted_poses, corrections):
        pose_errors = []
        for corner in camera.tagged_corners(observation):
            world_xy = arena_corner_for_walls(corner["walls"], arena_x, arena_y)
            predicted = project_point(matrix, robot_point(
                world_xy, corner["edge_height"], fitted))
            pose_errors.append(float(np.linalg.norm(predicted - corner["pixel"])))
        errors.extend(pose_errors)
        pose_summary = (error_summary(np.asarray(pose_errors)) if pose_errors else {
            "median_error_px": None, "p90_error_px": None,
            "rmse_error_px": None, "max_error_px": None,
        })
        reports.append({
            "id": observation["id"],
            "measured_pose": {"x_m": measured[0], "y_m": measured[1],
                              "yaw_deg": math.degrees(measured[2])},
            "correction": {"x_m": correction[0], "y_m": correction[1],
                           "yaw_deg": math.degrees(correction[2])},
            "corner_count": len(pose_errors),
            "median_corner_error_px": pose_summary["median_error_px"],
            "p90_corner_error_px": pose_summary["p90_error_px"],
            "rmse_corner_error_px": pose_summary["rmse_error_px"],
            "max_corner_error_px": pose_summary["max_error_px"],
        })
    errors = np.asarray(errors)
    decomposed_k, rotation, camera_centre, *_ = cv2.decomposeProjectionMatrix(matrix)
    decomposed_k /= decomposed_k[2, 2]
    camera_centre = (camera_centre[:3] / camera_centre[3]).ravel()
    report = {
        "projection_matrix": matrix.tolist(),
        "gauge_fixed_flat_index": gauge_index,
        "decomposition": {
            "K": decomposed_k.tolist(),
            "rotation_robot_to_camera": rotation.tolist(),
            "camera_centre_robot_m": camera_centre.tolist(),
            "left_3x3_condition_number": float(np.linalg.cond(matrix[:, :3])),
        },
        "fixed_measured_pose_fit": error_summary(fixed_errors),
        "fit": {
            "success": bool(joint_fit.success), "message": joint_fit.message,
            "metric": "visible_corner_reprojection", "loss": "linear_squared",
            **error_summary(errors),
        },
        "poses": reports,
    }
    (output_dir / "fit.json").write_text(json.dumps(report, indent=2) + "\n",
                                          encoding="utf-8")
    storage = cv2.FileStorage(str(output_dir / "projective_camera.yaml"),
                              cv2.FILE_STORAGE_WRITE)
    try:
        storage.write("P_robot_to_rectified", matrix)
        storage.write("decomposed_K", decomposed_k)
        storage.write("camera_centre_robot_m", camera_centre.reshape(3, 1))
        storage.write("fence_height_m", args.fence_height)
    finally:
        storage.release()
    render(output_dir, observations, fitted_poses, matrix, arena_x, arena_y,
           args.fence_height, args.pixel_sigma)

    print("\nP_robot_to_rectified (scale gauge fixed at flat index {}):".format(gauge_index))
    print(matrix)
    print("corner error: median={:.2f}px p90={:.2f}px rmse={:.2f}px max={:.2f}px".format(
        report["fit"]["median_error_px"], report["fit"]["p90_error_px"],
        report["fit"]["rmse_error_px"], report["fit"]["max_error_px"]))
    print("wrote {}".format(output_dir))
    return 0 if joint_fit.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
