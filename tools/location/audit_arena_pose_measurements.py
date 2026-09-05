#!/usr/bin/env python3
"""Visualize measured arena poses against reviewed image corners."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

import regress_arena_camera as camera
import regress_arena_projective as projective


def pose_corner_residual(pose, observation, matrix, arena_x, arena_y):
    residuals = []
    for corner in camera.tagged_corners(observation):
        world_xy = projective.arena_corner_for_walls(corner["walls"], arena_x, arena_y)
        predicted = projective.project_point(
            matrix, projective.robot_point(world_xy, corner["edge_height"], pose))
        residuals.extend(predicted - corner["pixel"])
    return np.asarray(residuals)


def fit_matrix(observations, poses, initial_parameters, decode,
               arena_x, arena_y, pixel_sigma, position_sigma, yaw_sigma,
               position_limit, yaw_limit):
    fixed = least_squares(
        projective.corner_residual, initial_parameters,
        args=(decode, observations, poses, arena_x, arena_y, pixel_sigma),
        loss="linear", x_scale="jac", max_nfev=1500)
    count = len(observations)
    lower = np.tile((-position_limit, -position_limit, -yaw_limit), count)
    upper = np.tile((position_limit, position_limit, yaw_limit), count)
    initial = np.concatenate((fixed.x, np.zeros(3 * count)))
    joint = least_squares(
        projective.joint_residual, initial,
        bounds=(np.concatenate((np.full(11, -np.inf), lower)),
                np.concatenate((np.full(11, np.inf), upper))),
        args=(decode, observations, poses, arena_x, arena_y, pixel_sigma,
              position_sigma, yaw_sigma),
        loss="linear", x_scale="jac", max_nfev=2000)
    return decode(joint.x[:11])


def draw_edges(image, matrix, pose, edges, colour, thickness):
    for edge in edges:
        projected, visible = projective.project_world(matrix, edge, pose)
        height, width = image.shape[:2]
        inside = (visible & (projected[:, 0] >= 0) & (projected[:, 0] < width) &
                  (projected[:, 1] >= 0) & (projected[:, 1] < height))
        indices = np.flatnonzero(inside)
        for run in np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1):
            if len(run) >= 2:
                cv2.polylines(image, [np.rint(projected[run]).astype(np.int32)],
                              False, colour, thickness, cv2.LINE_AA)


def draw_pose_inset(image, measured, inferred, arena_x, arena_y):
    left, top, width, height = 24, image.shape[0] - 210, 300, 180
    cv2.rectangle(image, (left, top), (left + width, top + height),
                  (248, 248, 248), -1)
    cv2.rectangle(image, (left, top), (left + width, top + height),
                  (30, 30, 30), 2)
    margin = 14

    def point(pose):
        return (round(left + margin + pose[0] / arena_x * (width - 2 * margin)),
                round(top + height - margin - pose[1] / arena_y * (height - 2 * margin)))

    def heading(pose, colour):
        origin = point(pose)
        scale = 35
        tip = (round(origin[0] + scale * math.cos(pose[2])),
               round(origin[1] - scale * math.sin(pose[2])))
        cv2.arrowedLine(image, origin, tip, colour, 3, cv2.LINE_AA, tipLength=.3)
        cv2.circle(image, origin, 5, colour, -1, cv2.LINE_AA)

    heading(measured, (0, 0, 255))
    if inferred is not None:
        heading(inferred, (0, 190, 0))


def render_pose(image, observation, measured, inferred, matrix,
                arena_x, arena_y, fence_height, report):
    view = image.copy()
    edges = camera.arena_edges(arena_x, arena_y, fence_height)
    draw_edges(view, matrix, measured, edges, (0, 0, 255), 2)
    if inferred is not None:
        draw_edges(view, matrix, inferred, edges, (0, 190, 0), 2)
    for corner in camera.tagged_corners(observation):
        colour = (255, 255, 0) if corner["edge_height"] else (0, 165, 255)
        observed = tuple(np.rint(corner["pixel"]).astype(int))
        cv2.circle(view, observed, 5, colour, -1, cv2.LINE_AA)
        for pose, marker_colour, marker_type in (
                (measured, (0, 0, 255), cv2.MARKER_TILTED_CROSS),
                (inferred, (0, 190, 0), cv2.MARKER_CROSS)):
            if pose is None:
                continue
            world_xy = projective.arena_corner_for_walls(
                corner["walls"], arena_x, arena_y)
            predicted = projective.project_point(
                matrix, projective.robot_point(world_xy, corner["edge_height"], pose))
            cv2.drawMarker(view, tuple(np.rint(predicted).astype(int)), marker_colour,
                           marker_type, 13, 2, cv2.LINE_AA)
    cv2.rectangle(view, (0, 0), (view.shape[1], 82), (20, 20, 20), -1)
    cv2.putText(view, "pose {:02d}: measured red X/lines; inferred green +/lines; tags cyan/orange".format(
                observation["id"]), (20, 28), cv2.FONT_HERSHEY_SIMPLEX, .60,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(view, report, (20, 59), cv2.FONT_HERSHEY_SIMPLEX, .53,
                (255, 255, 255), 1, cv2.LINE_AA)
    draw_pose_inset(view, measured, inferred, arena_x, arena_y)
    return view


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
    entries = [entry for entry in json.loads(args.tags.read_text(encoding="utf-8"))["poses"]
               if entry.get("reviewed")]
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    _, _, rectified_matrix, width, height = camera.load_calibration(calibration_path)
    arena_x = float(session["arena"]["x_length_m"])
    arena_y = float(session["arena"]["y_length_m"])
    output_dir = args.output_dir or args.session / "pose-measurement-audit"
    output_dir.mkdir(parents=True, exist_ok=True)

    observations, poses, images = [], [], []
    for entry in entries:
        segments = (camera.tagged_segments(entry.get("upper", []), args.fence_height) +
                    camera.tagged_segments(entry.get("lower", []), 0.0))
        observations.append({"id": entry["id"], "segments": segments})
        poses.append((entry["x_m"], entry["y_m"], math.radians(entry["yaw_deg"])))
        image = cv2.imread(str(args.session / entry["rectified_image"]), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise ValueError("cannot load pose {} image".format(entry["id"]))
        images.append(image)
    poses = np.asarray(poses, np.float64)
    initial_mount = np.array((-.04, 0.0, .145, .5, 30.4, 0.0))
    camera.assign_walls(observations, poses, initial_mount,
                        rectified_matrix, arena_x, arena_y)
    initial_parameters, decode, _ = projective.matrix_codec(
        projective.physical_matrix(initial_mount, rectified_matrix))

    reports, rendered = [], []
    for held, (observation, measured, image) in enumerate(zip(observations, poses, images)):
        train_observations = [item for index, item in enumerate(observations) if index != held]
        train_poses = np.delete(poses, held, axis=0)
        matrix = fit_matrix(
            train_observations, train_poses, initial_parameters, decode,
            arena_x, arena_y, args.pixel_sigma, args.position_sigma,
            math.radians(args.yaw_sigma_deg), args.position_correction_limit,
            math.radians(args.yaw_correction_limit_deg))
        corner_count = len(camera.tagged_corners(observation))
        inferred = None
        if corner_count >= 2:
            lower = np.array((0.0, 0.0, measured[2] - math.radians(30.0)))
            upper = np.array((arena_x, arena_y, measured[2] + math.radians(30.0)))
            solution = least_squares(
                pose_corner_residual, measured, bounds=(lower, upper),
                args=(observation, matrix, arena_x, arena_y),
                loss="linear", max_nfev=1000)
            inferred = solution.x
        measured_residual = pose_corner_residual(
            measured, observation, matrix, arena_x, arena_y).reshape(-1, 2)
        measured_rmse = float(np.sqrt(np.mean(np.sum(np.square(measured_residual), axis=1))))
        if inferred is None:
            text = "measured ({:.3f},{:.3f},{:.1f}deg), RMSE {:.1f}px; inference underdetermined".format(
                measured[0], measured[1], math.degrees(measured[2]), measured_rmse)
            item = {"id": observation["id"], "corner_count": corner_count,
                    "measured_rmse_px": measured_rmse, "inferred_pose": None}
        else:
            inferred_residual = pose_corner_residual(
                inferred, observation, matrix, arena_x, arena_y).reshape(-1, 2)
            inferred_rmse = float(np.sqrt(np.mean(np.sum(np.square(inferred_residual), axis=1))))
            delta = inferred - measured
            text = "measured ({:.3f},{:.3f},{:.1f}deg), delta ({:+.3f},{:+.3f},{:+.1f}deg), RMSE {:.1f}->{:.1f}px".format(
                measured[0], measured[1], math.degrees(measured[2]),
                delta[0], delta[1], math.degrees(delta[2]), measured_rmse, inferred_rmse)
            item = {
                "id": observation["id"], "corner_count": corner_count,
                "measured_rmse_px": measured_rmse,
                "inferred_rmse_px": inferred_rmse,
                "measured_pose": {"x_m": measured[0], "y_m": measured[1],
                                  "yaw_deg": math.degrees(measured[2])},
                "inferred_pose": {"x_m": inferred[0], "y_m": inferred[1],
                                  "yaw_deg": math.degrees(inferred[2])},
                "delta": {"x_m": delta[0], "y_m": delta[1],
                          "yaw_deg": math.degrees(delta[2])},
            }
        reports.append(item)
        view = render_pose(image, observation, measured, inferred, matrix,
                           arena_x, arena_y, args.fence_height, text)
        cv2.imwrite(str(output_dir / "pose-{:02d}.jpg".format(observation["id"])),
                    view, [cv2.IMWRITE_JPEG_QUALITY, 94])
        rendered.append(cv2.resize(view, (640, 360), interpolation=cv2.INTER_AREA))
        print(text)

    rows = []
    for index in range(0, len(rendered), 2):
        pair = rendered[index:index + 2]
        if len(pair) == 1:
            pair.append(np.full_like(pair[0], 245))
        rows.append(np.hstack(pair))
    cv2.imwrite(str(output_dir / "contact-sheet.jpg"), np.vstack(rows),
                [cv2.IMWRITE_JPEG_QUALITY, 94])
    (output_dir / "audit.json").write_text(json.dumps({"poses": reports}, indent=2) + "\n",
                                             encoding="utf-8")
    print("wrote {}".format(output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
