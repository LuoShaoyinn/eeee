#!/usr/bin/env python3
"""Fit the rigid camera mount from measured arena poses and blue fence edges."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


MOUNT_NAMES = ("forward_m", "left_m", "height_m", "yaw_deg", "pitch_deg", "roll_deg")


def load_calibration(path: Path):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open calibration {}".format(path))
    try:
        camera_matrix = storage.getNode("K").mat()
        distortion = storage.getNode("D").mat()
        rectified_matrix = storage.getNode("rectified_K").mat()
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
    finally:
        storage.release()
    if (camera_matrix is None or distortion is None or rectified_matrix is None or
            width <= 0 or height <= 0):
        raise ValueError("calibration must contain K, D, rectified_K, image_width, and image_height")
    return (camera_matrix.astype(np.float64), distortion.astype(np.float64),
            rectified_matrix.astype(np.float64), width, height)


def load_blue_ranges(path: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    profile = json.loads(path.read_text(encoding="utf-8"))
    ranges = profile.get("blue_fence")
    if not ranges:
        raise ValueError("{} has no blue_fence ranges".format(path))
    return [(np.asarray(low, np.uint8), np.asarray(high, np.uint8)) for low, high in ranges]


def median_image(session_dir: Path, pose: dict) -> np.ndarray:
    frames = []
    for item in pose["images"]:
        frame = cv2.imread(str(session_dir / item["path"]), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("cannot read {}".format(session_dir / item["path"]))
        frames.append(frame)
    if not frames:
        raise ValueError("pose {} has no images".format(pose["id"]))
    shape = frames[0].shape
    if any(frame.shape != shape for frame in frames):
        raise ValueError("pose {} contains mixed image sizes".format(pose["id"]))
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def blue_mask(image: np.ndarray, ranges, close_size: int) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape[:2], np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    count, labels, statistics, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return np.zeros_like(mask)
    largest = 1 + int(np.argmax(statistics[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest, 255, 0).astype(np.uint8)


def contour_pixels(mask: np.ndarray, step: int, margin: int, min_span: int):
    upper, lower = [], []
    height, width = mask.shape
    for x in range(margin, width - margin, step):
        ys = np.flatnonzero(mask[:, x])
        if ys.size == 0:
            continue
        top, bottom = int(ys[0]), int(ys[-1])
        if top <= margin or bottom >= height - margin or bottom - top < min_span:
            continue
        upper.append((float(x), float(top)))
        lower.append((float(x), float(bottom)))
    return np.asarray(upper, np.float64), np.asarray(lower, np.float64)


def pixel_rays(points: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray) -> np.ndarray:
    if not len(points):
        return np.empty((0, 3), np.float64)
    normalized = cv2.fisheye.undistortPoints(points.reshape(-1, 1, 2), camera_matrix, distortion)
    rays = np.column_stack((normalized.reshape(-1, 2), np.ones(len(points))))
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def rectified_pixel_rays(points: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    if not len(points):
        return np.empty((0, 3), np.float64)
    inverse = np.linalg.inv(camera_matrix)
    homogeneous = np.column_stack((points, np.ones(len(points))))
    rays = homogeneous @ inverse.T
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def sample_polyline(points, spacing_px: float = 20.0) -> np.ndarray:
    points = np.asarray(points, np.float64)
    if len(points) < 2:
        return np.empty((0, 2), np.float64)
    samples = []
    for index, endpoints in enumerate(zip(points[:-1], points[1:])):
        endpoints = np.asarray(endpoints, np.float64)
        count = max(2, int(np.linalg.norm(endpoints[1] - endpoints[0]) / spacing_px) + 1)
        fraction = np.linspace(0.0, 1.0, count)
        segment = np.outer(1.0 - fraction, endpoints[0]) + np.outer(fraction, endpoints[1])
        samples.append(segment if index == 0 else segment[1:])
    return np.vstack(samples) if samples else np.empty((0, 2), np.float64)


def tagged_segments(points, edge_height: float, spacing_px: float = 20.0):
    points = np.asarray(points, np.float64)
    result = []
    for endpoints in zip(points[:-1], points[1:]):
        endpoints = np.asarray(endpoints, np.float64)
        count = max(2, int(np.linalg.norm(endpoints[1] - endpoints[0]) / spacing_px) + 1)
        fraction = np.linspace(0.0, 1.0, count)
        pixels = np.outer(1.0 - fraction, endpoints[0]) + np.outer(fraction, endpoints[1])
        result.append({
            "edge_height": edge_height,
            "pixels": pixels,
            "endpoints": endpoints,
        })
    return result


def camera_frame(mount: np.ndarray, robot_yaw_rad: float):
    forward_m, left_m, height_m, yaw_deg, pitch_deg, roll_deg = mount
    robot_forward = np.array([math.cos(robot_yaw_rad), math.sin(robot_yaw_rad), 0.0])
    robot_left = np.array([-math.sin(robot_yaw_rad), math.cos(robot_yaw_rad), 0.0])
    centre_offset = forward_m * robot_forward + left_m * robot_left + np.array([0.0, 0.0, height_m])

    heading = robot_yaw_rad + math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    horizontal = np.array([math.cos(heading), math.sin(heading), 0.0])
    horizontal_left = np.array([-math.sin(heading), math.cos(heading), 0.0])
    z_camera = math.cos(pitch) * horizontal - math.sin(pitch) * np.array([0.0, 0.0, 1.0])
    x_zero = -horizontal_left
    y_zero = np.cross(z_camera, x_zero)
    x_camera = math.cos(roll) * x_zero + math.sin(roll) * y_zero
    y_camera = -math.sin(roll) * x_zero + math.cos(roll) * y_zero
    rotation_world_from_camera = np.column_stack((x_camera, y_camera, z_camera))
    return centre_offset, rotation_world_from_camera


def projected_wall_line(mount, pose, wall_index, edge_height, camera_matrix,
                        arena_x, arena_y):
    corners = ((0.0, 0.0), (arena_x, 0.0), (arena_x, arena_y), (0.0, arena_y))
    first = np.array((*corners[wall_index], edge_height), np.float64)
    second = np.array((*corners[(wall_index + 1) % 4], edge_height), np.float64)
    centre_offset, rotation_world_from_camera = camera_frame(mount, pose[2])
    camera_centre = np.array([pose[0], pose[1], 0.0]) + centre_offset
    rotation_camera_from_world = rotation_world_from_camera.T
    first_h = camera_matrix @ (rotation_camera_from_world @ (first - camera_centre))
    second_h = camera_matrix @ (rotation_camera_from_world @ (second - camera_centre))
    line = np.cross(first_h, second_h)
    normal = np.linalg.norm(line[:2])
    if normal < 1e-9:
        return np.array([0.0, 0.0, 1e6])
    return line / normal


def assign_walls(observations, poses, mount, camera_matrix, arena_x, arena_y):
    def segment_cost(segment, wall_index):
        pixels = segment["pixels"]
        direction = pixels[-1] - pixels[0]
        observed_angle = math.atan2(direction[1], direction[0])
        line = projected_wall_line(
            mount, pose, wall_index, segment["edge_height"], camera_matrix,
            arena_x, arena_y)
        distance = float(np.mean(np.abs(pixels @ line[:2] + line[2])))
        line_angle = math.atan2(-line[0], line[1])
        angle_delta = abs((observed_angle - line_angle + math.pi / 2) % math.pi -
                          math.pi / 2)
        return distance + 40.0 * angle_delta

    def best_sequence(segment_groups):
        count = len(segment_groups[0])
        candidates = []
        for start in range(4):
            for direction in (-1, 1):
                sequence = [(start + direction * index) % 4 for index in range(count)]
                cost = sum(segment_cost(segment, sequence[index])
                           for group in segment_groups
                           for index, segment in enumerate(group))
                candidates.append((cost, sequence))
        return min(candidates, key=lambda item: item[0])[1]

    assignments = []
    for observation, pose in zip(observations, poses):
        pose_assignments = []
        groups = []
        for edge_height in dict.fromkeys(
                segment["edge_height"] for segment in observation["segments"]):
            groups.append([segment for segment in observation["segments"]
                           if segment["edge_height"] == edge_height])

        # The upper and lower contours describe the same vertical wall faces.
        # Scoring them independently can assign a paired edge to different arena sides.
        if len(groups) > 1 and len({len(group) for group in groups}) == 1:
            sequence = best_sequence(groups)
            grouped_sequences = [(group, sequence) for group in groups]
        else:
            grouped_sequences = [(group, best_sequence([group])) for group in groups]

        for group, sequence in grouped_sequences:
            for segment, wall in zip(group, sequence):
                segment["wall"] = wall
                pose_assignments.append(wall)
        assignments.append(pose_assignments)
    return assignments


def tagged_mount_residual(mount, observations, poses, camera_matrix,
                          arena_x, arena_y, pixel_sigma):
    residuals = []
    for observation, pose in zip(observations, poses):
        for corner in tagged_corners(observation):
            first = projected_wall_line(mount, pose, corner["walls"][0],
                                        corner["edge_height"], camera_matrix,
                                        arena_x, arena_y)
            second = projected_wall_line(mount, pose, corner["walls"][1],
                                         corner["edge_height"], camera_matrix,
                                         arena_x, arena_y)
            intersection = np.cross(first, second)
            if abs(intersection[2]) < 1e-9:
                residuals.extend((1e3, 1e3))
            else:
                predicted = intersection[:2] / intersection[2]
                residuals.extend((predicted - corner["pixel"]) / pixel_sigma)
    return np.asarray(residuals)


def tagged_corners(observation):
    corners = []
    heights = []
    for segment in observation["segments"]:
        if segment["edge_height"] not in heights:
            heights.append(segment["edge_height"])
    for edge_height in heights:
        segments = [segment for segment in observation["segments"]
                    if segment["edge_height"] == edge_height]
        for first, second in zip(segments[:-1], segments[1:]):
            corners.append({
                "edge_height": edge_height,
                "pixel": 0.5 * (first["endpoints"][1] + second["endpoints"][0]),
                "walls": (first["wall"], second["wall"]),
            })
    return corners


def tagged_joint_residual(parameters, observations, measured_poses, camera_matrix,
                          arena_x, arena_y, pixel_sigma, position_sigma, yaw_sigma):
    mount = parameters[:6]
    corrections = parameters[6:].reshape(-1, 3)
    poses = measured_poses + corrections
    image = tagged_mount_residual(mount, observations, poses, camera_matrix,
                                  arena_x, arena_y, pixel_sigma)
    priors = corrections.copy()
    priors[:, :2] /= position_sigma
    priors[:, 2] /= yaw_sigma
    return np.concatenate((image, priors.ravel()))


def adjusted_rectified_matrix(base_matrix, intrinsics):
    matrix = base_matrix.copy()
    matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2] = intrinsics
    return matrix


def tagged_calibration_residual(parameters, observations, poses, base_matrix,
                                arena_x, arena_y, pixel_sigma,
                                focal_sigma_fraction, centre_sigma_px):
    mount = parameters[:6]
    intrinsics = parameters[6:10]
    matrix = adjusted_rectified_matrix(base_matrix, intrinsics)
    image = tagged_mount_residual(mount, observations, poses, matrix,
                                  arena_x, arena_y, pixel_sigma)
    base = np.array((base_matrix[0, 0], base_matrix[1, 1],
                     base_matrix[0, 2], base_matrix[1, 2]))
    priors = intrinsics - base
    priors[:2] /= base[:2] * focal_sigma_fraction
    priors[2:] /= centre_sigma_px
    return np.concatenate((image, priors))


def tagged_calibration_joint_residual(parameters, observations, measured_poses,
                                      base_matrix, arena_x, arena_y, pixel_sigma,
                                      focal_sigma_fraction, centre_sigma_px,
                                      position_sigma, yaw_sigma):
    global_parameters = parameters[:10]
    corrections = parameters[10:].reshape(-1, 3)
    poses = measured_poses + corrections
    calibration = tagged_calibration_residual(
        global_parameters, observations, poses, base_matrix, arena_x, arena_y,
        pixel_sigma, focal_sigma_fraction, centre_sigma_px)
    priors = corrections.copy()
    priors[:, :2] /= position_sigma
    priors[:, 2] /= yaw_sigma
    return np.concatenate((calibration, priors.ravel()))


def tagged_jacobian_sparsity(observations):
    image_rows = 2 * sum(len(tagged_corners(item)) for item in observations)
    pose_count = len(observations)
    sparsity = lil_matrix((image_rows + 3 * pose_count, 6 + 3 * pose_count), dtype=np.int8)
    row = 0
    for index, observation in enumerate(observations):
        count = 2 * len(tagged_corners(observation))
        sparsity[row:row + count, :6] = 1
        sparsity[row:row + count, 6 + 3 * index:9 + 3 * index] = 1
        row += count
    for index in range(pose_count):
        sparsity[image_rows + 3 * index:image_rows + 3 * index + 3,
                 6 + 3 * index:9 + 3 * index] = 1
    return sparsity.tocsr()


def distance_to_rectangle(points: np.ndarray, arena_x: float, arena_y: float) -> np.ndarray:
    x, y = points[:, 0], points[:, 1]
    horizontal_x = np.clip(x, 0.0, arena_x)
    vertical_y = np.clip(y, 0.0, arena_y)
    distances = np.column_stack((
        np.hypot(x - horizontal_x, y),
        np.hypot(x - horizontal_x, y - arena_y),
        np.hypot(x, y - vertical_y),
        np.hypot(x - arena_x, y - vertical_y),
    ))
    return np.min(distances, axis=1)


def intersect_edge(rays_camera: np.ndarray, edge_height: float, mount: np.ndarray,
                   pose: np.ndarray, arena_x: float, arena_y: float):
    centre_offset, rotation = camera_frame(mount, pose[2])
    robot_centre = np.array([pose[0], pose[1], 0.0])
    camera_centre = robot_centre + centre_offset
    rays_world = rays_camera @ rotation.T
    denominator = rays_world[:, 2]
    scale = np.divide(edge_height - camera_centre[2], denominator,
                      out=np.full_like(denominator, -1.0), where=np.abs(denominator) > 1e-7)
    valid = scale > 0.0
    points = camera_centre + scale[:, None] * rays_world
    distance = distance_to_rectangle(points[:, :2], arena_x, arena_y)
    distance[~valid] = 1.0
    return points, valid, distance


def mount_residual(mount, observations, poses, arena_x, arena_y, fence_height, edge_sigma):
    residuals = []
    for observation, pose in zip(observations, poses):
        _, _, upper = intersect_edge(observation["upper_rays"], fence_height, mount, pose,
                                     arena_x, arena_y)
        _, _, lower = intersect_edge(observation["lower_rays"], 0.0, mount, pose,
                                     arena_x, arena_y)
        residuals.extend(upper / edge_sigma)
        residuals.extend(lower / edge_sigma)
    return np.asarray(residuals)


def joint_residual(parameters, observations, measured_poses, arena_x, arena_y, fence_height,
                   edge_sigma, position_sigma, yaw_sigma):
    mount = parameters[:6]
    corrections = parameters[6:].reshape(-1, 3)
    poses = measured_poses + corrections
    edge = mount_residual(mount, observations, poses, arena_x, arena_y, fence_height, edge_sigma)
    priors = corrections.copy()
    priors[:, :2] /= position_sigma
    priors[:, 2] /= yaw_sigma
    return np.concatenate((edge, priors.ravel()))


def joint_jacobian_sparsity(observations):
    edge_rows = sum(len(item["upper_rays"]) + len(item["lower_rays"])
                    for item in observations)
    pose_count = len(observations)
    sparsity = lil_matrix((edge_rows + 3 * pose_count, 6 + 3 * pose_count), dtype=np.int8)
    row = 0
    for index, observation in enumerate(observations):
        count = len(observation["upper_rays"]) + len(observation["lower_rays"])
        sparsity[row:row + count, :6] = 1
        sparsity[row:row + count, 6 + 3 * index:9 + 3 * index] = 1
        row += count
    for index in range(pose_count):
        sparsity[edge_rows + 3 * index:edge_rows + 3 * index + 3,
                 6 + 3 * index:9 + 3 * index] = 1
    return sparsity.tocsr()


def render_extraction(path: Path, image: np.ndarray, upper: np.ndarray, lower: np.ndarray) -> None:
    view = image.copy()
    for point in upper:
        cv2.circle(view, tuple(np.rint(point).astype(int)), 3, (255, 255, 0), -1)
    for point in lower:
        cv2.circle(view, tuple(np.rint(point).astype(int)), 3, (0, 165, 255), -1)
    cv2.putText(view, "cyan: upper blue edge; orange: lower blue edge",
                (24, 34), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), view, [cv2.IMWRITE_JPEG_QUALITY, 94])


def project_world(points_world: np.ndarray, mount: np.ndarray, pose: np.ndarray,
                  camera_matrix: np.ndarray, distortion: np.ndarray):
    centre_offset, rotation_world_from_camera = camera_frame(mount, pose[2])
    camera_centre = np.array([pose[0], pose[1], 0.0]) + centre_offset
    rotation_camera_from_world = rotation_world_from_camera.T
    translation = -rotation_camera_from_world @ camera_centre
    rotation_vector, _ = cv2.Rodrigues(rotation_camera_from_world)
    if distortion is None:
        projected, _ = cv2.projectPoints(
            points_world, rotation_vector, translation.reshape(3, 1), camera_matrix, None)
    else:
        projected, _ = cv2.fisheye.projectPoints(
            points_world.reshape(1, -1, 3), rotation_vector, translation.reshape(3, 1),
            camera_matrix, distortion)
    camera_points = (rotation_camera_from_world @ points_world.T).T + translation
    return projected.reshape(-1, 2), camera_points[:, 2] > 0.0


def arena_edges(arena_x: float, arena_y: float, height: float, samples: int = 160):
    corners = ((0.0, 0.0), (arena_x, 0.0), (arena_x, arena_y), (0.0, arena_y))
    edges = []
    for start, end in zip(corners, corners[1:] + corners[:1]):
        fraction = np.linspace(0.0, 1.0, samples)
        xy = np.outer(1.0 - fraction, start) + np.outer(fraction, end)
        edges.append(np.column_stack((xy, np.zeros(samples))))
        edges.append(np.column_stack((xy, np.full(samples, height))))
    return edges


def draw_polyline(image, projected, visible, colour):
    height, width = image.shape[:2]
    inside = (visible & (projected[:, 0] >= 0) & (projected[:, 0] < width) &
              (projected[:, 1] >= 0) & (projected[:, 1] < height))
    indices = np.flatnonzero(inside)
    if not len(indices):
        return
    runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    for run in runs:
        if len(run) >= 2:
            cv2.polylines(image, [np.rint(projected[run]).astype(np.int32)], False,
                          colour, 3, cv2.LINE_AA)


def render_diagnostics(output_dir, observations, poses, mount, camera_matrix, distortion,
                       arena_x, arena_y, fence_height, pixel_sigma):
    overlay_dir = output_dir / "overlays"
    bev_dir = output_dir / "bev"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    bev_dir.mkdir(parents=True, exist_ok=True)
    edges = arena_edges(arena_x, arena_y, fence_height)
    scale = 280
    border = round(0.5 * scale)
    canvas_width = round(arena_x * scale) + border * 2
    canvas_height = round(arena_y * scale) + border * 2

    for observation, pose in zip(observations, poses):
        overlay = observation["image"].copy()
        for point in observation["upper_pixels"]:
            cv2.circle(overlay, tuple(np.rint(point).astype(int)), 2, (255, 255, 0), -1)
        for point in observation["lower_pixels"]:
            cv2.circle(overlay, tuple(np.rint(point).astype(int)), 2, (0, 165, 255), -1)
        for key, colour in (("upper_corners", (255, 255, 0)),
                            ("lower_corners", (0, 165, 255))):
            for point in observation.get(key, []):
                centre = tuple(np.rint(point).astype(int))
                cv2.circle(overlay, centre, round(pixel_sigma), colour, 1, cv2.LINE_AA)
                cv2.circle(overlay, centre, 4, colour, -1, cv2.LINE_AA)
        for index, edge in enumerate(edges):
            projected, visible = project_world(edge, mount, pose, camera_matrix, distortion)
            draw_polyline(overlay, projected, visible, (255, 0, 255) if index % 2 else (0, 255, 0))
        for corner in tagged_corners(observation) if "segments" in observation else []:
            first = projected_wall_line(mount, pose, corner["walls"][0],
                                        corner["edge_height"], camera_matrix,
                                        arena_x, arena_y)
            second = projected_wall_line(mount, pose, corner["walls"][1],
                                         corner["edge_height"], camera_matrix,
                                         arena_x, arena_y)
            intersection = np.cross(first, second)
            if abs(intersection[2]) < 1e-9:
                continue
            predicted = intersection[:2] / intersection[2]
            observed_point = tuple(np.rint(corner["pixel"]).astype(int))
            predicted_point = tuple(np.rint(predicted).astype(int))
            error = np.linalg.norm(predicted - corner["pixel"])
            cv2.line(overlay, observed_point, predicted_point, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.drawMarker(overlay, predicted_point, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
            cv2.putText(overlay, "{:.1f}px".format(error),
                        (observed_point[0] + 7, observed_point[1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, "circles: observed corners (+/-{:.0f}px); crosses: fitted corners".format(
                    pixel_sigma),
                    (24, 34), cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(overlay_dir / "pose-{:02d}.jpg".format(observation["id"])), overlay,
                    [cv2.IMWRITE_JPEG_QUALITY, 94])

        canvas = np.full((canvas_height, canvas_width, 3), 245, np.uint8)
        def map_point(point):
            return (round(border + point[0] * scale), round(canvas_height - border - point[1] * scale))
        cv2.rectangle(canvas, map_point((0.0, arena_y)), map_point((arena_x, 0.0)), (20, 20, 20), 3)

        uncertainty_layer = canvas.copy()
        angles = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
        offsets = pixel_sigma * np.column_stack((np.cos(angles), np.sin(angles)))
        for vertices, edge_height, colour in (
                (observation.get("upper_corners", []), fence_height, (255, 190, 255)),
                (observation.get("lower_corners", []), 0.0, (120, 205, 255))):
            for vertex in vertices:
                rays = rectified_pixel_rays(vertex[None, :] + offsets, camera_matrix)
                points, valid, _ = intersect_edge(rays, edge_height, mount, pose,
                                                  arena_x, arena_y)
                points = points[valid, :2]
                if len(points) < 3:
                    continue
                points[:, 0] = np.clip(points[:, 0], -0.5, arena_x + 0.5)
                points[:, 1] = np.clip(points[:, 1], -0.5, arena_y + 0.5)
                polygon = np.asarray([map_point(point) for point in points], np.int32)
                polygon = cv2.convexHull(polygon)
                cv2.fillConvexPoly(uncertainty_layer, polygon, colour, cv2.LINE_AA)
                cv2.polylines(canvas, [polygon], True, colour, 1, cv2.LINE_AA)
                centre_ray = rectified_pixel_rays(vertex[None, :], camera_matrix)
                centre, centre_valid, _ = intersect_edge(centre_ray, edge_height, mount, pose,
                                                         arena_x, arena_y)
                if centre_valid[0]:
                    cv2.circle(canvas, map_point(centre[0, :2]), 4, colour, -1, cv2.LINE_AA)
        cv2.addWeighted(uncertainty_layer, 0.30, canvas, 0.70, 0.0, canvas)

        for rays, edge_height, colour in ((observation["upper_rays"], fence_height, (255, 0, 255)),
                                          (observation["lower_rays"], 0.0, (0, 140, 255))):
            points, valid, _ = intersect_edge(rays, edge_height, mount, pose, arena_x, arena_y)
            for point in points[valid]:
                if -0.5 <= point[0] <= arena_x + 0.5 and -0.5 <= point[1] <= arena_y + 0.5:
                    cv2.circle(canvas, map_point(point), 2, colour, -1)
        camera_xy = np.array([pose[0], pose[1]])
        origin = map_point(camera_xy)
        direction = np.array([math.cos(pose[2]), math.sin(pose[2])])
        tip = map_point(camera_xy + 0.20 * direction)
        cv2.arrowedLine(canvas, origin, tip, (0, 0, 220), 3, cv2.LINE_AA, tipLength=.3)
        cv2.putText(canvas, "pose {:02d}".format(observation["id"]), (15, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(canvas, "+/- {:.0f}px endpoint uncertainty".format(pixel_sigma), (15, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (70, 70, 70), 1, cv2.LINE_AA)
        cv2.imwrite(str(bev_dir / "pose-{:02d}.png".format(observation["id"])), canvas)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="directory containing poses.json")
    parser.add_argument("--hsv", type=Path, required=True, help="HSV profile containing blue_fence")
    parser.add_argument("--tags", type=Path,
                        help="reviewed rectified line tags; preferred over automatic raw contours")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fence-height", type=float, default=0.254)
    parser.add_argument("--sample-step", type=int, default=16)
    parser.add_argument("--edge-margin", type=int, default=3)
    parser.add_argument("--minimum-blue-span", type=int, default=12)
    parser.add_argument("--close-size", type=int, default=9)
    parser.add_argument("--edge-sigma", type=float, default=0.03)
    parser.add_argument("--pixel-sigma", type=float, default=3.0,
                        help="reviewed line uncertainty in rectified-image pixels")
    parser.add_argument("--position-sigma", type=float, default=0.02)
    parser.add_argument("--yaw-sigma-deg", type=float, default=1.0)
    parser.add_argument("--position-correction-limit", type=float, default=0.02)
    parser.add_argument("--yaw-correction-limit-deg", type=float, default=1.0)
    parser.add_argument("--refine-rectified-intrinsics", action="store_true",
                        help="fit constrained fx/fy/cx/cy corrections after fisheye rectification")
    parser.add_argument("--focal-prior-fraction", type=float, default=0.02)
    parser.add_argument("--centre-prior-px", type=float, default=8.0)
    parser.add_argument("--initial-forward", type=float, default=0.10)
    parser.add_argument("--initial-left", type=float, default=0.0)
    parser.add_argument("--initial-height", type=float, default=0.136)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--initial-pitch", type=float, default=29.9)
    parser.add_argument("--initial-roll", type=float, default=0.0)
    args = parser.parse_args()
    if args.close_size < 1 or args.close_size % 2 == 0:
        parser.error("--close-size must be a positive odd number")
    if min(args.fence_height, args.sample_step, args.edge_sigma, args.pixel_sigma,
           args.position_sigma, args.yaw_sigma_deg, args.position_correction_limit,
           args.yaw_correction_limit_deg, args.focal_prior_fraction,
           args.centre_prior_px) <= 0:
        parser.error("height, sampling, and uncertainty values must be positive")

    metadata_path = args.session / "poses.json"
    session = json.loads(metadata_path.read_text(encoding="utf-8"))
    accepted = [pose for pose in session["poses"] if pose.get("accepted")]
    if len(accepted) < 6:
        parser.error("need at least six accepted measured poses; got {}".format(len(accepted)))
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    camera_matrix, distortion, rectified_matrix, width, height = load_calibration(calibration_path)
    ranges = load_blue_ranges(args.hsv)
    arena_x = float(session["arena"]["x_length_m"])
    arena_y = float(session["arena"]["y_length_m"])
    output_dir = args.output_dir or args.session / "regression"
    output_dir.mkdir(parents=True, exist_ok=True)
    extraction_dir = output_dir / "extractions"
    extraction_dir.mkdir(parents=True, exist_ok=True)

    observations = []
    measured_poses = []
    diagnostic_matrix = camera_matrix
    diagnostic_distortion = distortion
    if args.tags:
        tags = json.loads(args.tags.read_text(encoding="utf-8"))
        tagged = [entry for entry in tags.get("poses", []) if entry.get("reviewed")]
        if len(tagged) < 6:
            parser.error("need at least six reviewed tagged poses; got {}".format(len(tagged)))
        diagnostic_matrix = rectified_matrix
        diagnostic_distortion = None
        for entry in tagged:
            image = cv2.imread(str(args.session / entry["rectified_image"]), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (height, width):
                raise ValueError("cannot read rectified tagged image for pose {}".format(entry["id"]))
            upper_vertices = np.asarray(entry.get("upper", []), np.float64)
            lower_vertices = np.asarray(entry.get("lower", []), np.float64)
            upper = sample_polyline(upper_vertices)
            lower = sample_polyline(lower_vertices)
            if len(upper) + len(lower) < 10:
                raise ValueError("pose {} has too few reviewed line samples".format(entry["id"]))
            observations.append({
                "id": entry["id"], "image": image,
                "upper_pixels": upper, "lower_pixels": lower,
                "upper_vertices": upper_vertices, "lower_vertices": lower_vertices,
                "upper_corners": upper_vertices[1:-1],
                "lower_corners": lower_vertices[1:-1],
                "upper_rays": rectified_pixel_rays(upper, rectified_matrix),
                "lower_rays": rectified_pixel_rays(lower, rectified_matrix),
                "segments": (tagged_segments(entry.get("upper", []), args.fence_height) +
                             tagged_segments(entry.get("lower", []), 0.0)),
            })
            measured_poses.append((entry["x_m"], entry["y_m"], math.radians(entry["yaw_deg"])))
            render_extraction(extraction_dir / "pose-{:02d}.jpg".format(entry["id"]),
                              image, upper, lower)
            print("pose {:02d}: reviewed upper={} lower={} samples".format(
                entry["id"], len(upper), len(lower)))
    else:
        for pose in accepted:
            image = median_image(args.session, pose)
            if image.shape[:2] != (height, width):
                raise ValueError("pose {} image size does not match calibration".format(pose["id"]))
            mask = blue_mask(image, ranges, args.close_size)
            upper, lower = contour_pixels(mask, args.sample_step, args.edge_margin,
                                          args.minimum_blue_span)
            if min(len(upper), len(lower)) < 20:
                raise ValueError("pose {} has too few blue fence samples ({})".format(
                    pose["id"], min(len(upper), len(lower))))
            observations.append({
                "id": pose["id"], "image": image,
                "upper_pixels": upper, "lower_pixels": lower,
                "upper_rays": pixel_rays(upper, camera_matrix, distortion),
                "lower_rays": pixel_rays(lower, camera_matrix, distortion),
            })
            measured_poses.append((pose["x_m"], pose["y_m"], math.radians(pose["yaw_deg"])))
            cv2.imwrite(str(output_dir / "mask-pose-{:02d}.png".format(pose["id"])), mask)
            render_extraction(extraction_dir / "pose-{:02d}.jpg".format(pose["id"]),
                              image, upper, lower)
            print("pose {:02d}: {} upper/lower samples".format(pose["id"], len(upper)))
    measured_poses = np.asarray(measured_poses, np.float64)

    initial_mount = np.array((args.initial_forward, args.initial_left, args.initial_height,
                              args.initial_yaw, args.initial_pitch, args.initial_roll), np.float64)
    mount_lower = np.array((-0.20, -0.25, 0.07, -25.0, 5.0, -15.0))
    mount_upper = np.array((0.30, 0.25, 0.25, 25.0, 65.0, 15.0))
    yaw_sigma = math.radians(args.yaw_sigma_deg)
    position_limit = args.position_correction_limit
    yaw_limit = math.radians(args.yaw_correction_limit_deg)
    correction_lower = np.tile((-position_limit, -position_limit, -yaw_limit), len(observations))
    correction_upper = np.tile((position_limit, position_limit, yaw_limit), len(observations))
    fitted_matrix = rectified_matrix
    if args.tags:
        assignments = assign_walls(observations, measured_poses, initial_mount,
                                   rectified_matrix, arena_x, arena_y)
        for observation, walls in zip(observations, assignments):
            print("pose {:02d}: wall assignments {}".format(observation["id"], walls))
        mount_scale = np.array((.10, .10, .04, 5.0, 10.0, 5.0))
        if args.refine_rectified_intrinsics:
            base_intrinsics = np.array((rectified_matrix[0, 0], rectified_matrix[1, 1],
                                        rectified_matrix[0, 2], rectified_matrix[1, 2]))
            global_initial = np.concatenate((initial_mount, base_intrinsics))
            intrinsic_lower = np.array((.95 * base_intrinsics[0], .95 * base_intrinsics[1],
                                        base_intrinsics[2] - 20.0, base_intrinsics[3] - 20.0))
            intrinsic_upper = np.array((1.05 * base_intrinsics[0], 1.05 * base_intrinsics[1],
                                        base_intrinsics[2] + 20.0, base_intrinsics[3] + 20.0))
            global_lower = np.concatenate((mount_lower, intrinsic_lower))
            global_upper = np.concatenate((mount_upper, intrinsic_upper))
            global_scale = np.concatenate((mount_scale, (8.0, 8.0, 8.0, 8.0)))
            fixed_fit = least_squares(
                tagged_calibration_residual, global_initial,
                bounds=(global_lower, global_upper),
                args=(observations, measured_poses, rectified_matrix, arena_x, arena_y,
                      args.pixel_sigma, args.focal_prior_fraction, args.centre_prior_px),
                x_scale=global_scale, loss="linear", max_nfev=1000, verbose=1)
            joint_initial = np.concatenate((fixed_fit.x, np.zeros(len(observations) * 3)))
            joint_fit = least_squares(
                tagged_calibration_joint_residual, joint_initial,
                bounds=(np.concatenate((global_lower, correction_lower)),
                        np.concatenate((global_upper, correction_upper))),
                args=(observations, measured_poses, rectified_matrix, arena_x, arena_y,
                      args.pixel_sigma, args.focal_prior_fraction, args.centre_prior_px,
                      args.position_sigma, yaw_sigma),
                x_scale=np.concatenate((global_scale,
                                        np.tile((args.position_sigma,
                                                 args.position_sigma, yaw_sigma),
                                                len(observations)))),
                loss="linear", max_nfev=2000, verbose=1)
            mount = joint_fit.x[:6]
            fitted_matrix = adjusted_rectified_matrix(rectified_matrix, joint_fit.x[6:10])
            corrections = joint_fit.x[10:].reshape(-1, 3)
        else:
            fixed_fit = least_squares(
                tagged_mount_residual, initial_mount, bounds=(mount_lower, mount_upper),
                args=(observations, measured_poses, rectified_matrix, arena_x, arena_y,
                      args.pixel_sigma),
                x_scale=mount_scale, loss="linear", max_nfev=500, verbose=1)
            joint_initial = np.concatenate((fixed_fit.x, np.zeros(len(observations) * 3)))
            joint_fit = least_squares(
                tagged_joint_residual, joint_initial,
                bounds=(np.concatenate((mount_lower, correction_lower)),
                        np.concatenate((mount_upper, correction_upper))),
                args=(observations, measured_poses, rectified_matrix, arena_x, arena_y,
                      args.pixel_sigma, args.position_sigma, yaw_sigma),
                jac_sparsity=tagged_jacobian_sparsity(observations), tr_solver="lsmr",
                x_scale=np.concatenate((mount_scale,
                                        np.tile((args.position_sigma,
                                                 args.position_sigma, yaw_sigma),
                                                len(observations)))),
                loss="linear", max_nfev=1000, verbose=1)
            mount = joint_fit.x[:6]
            corrections = joint_fit.x[6:].reshape(-1, 3)
    else:
        fixed_fit = least_squares(
            mount_residual, initial_mount, bounds=(mount_lower, mount_upper),
            args=(observations, measured_poses, arena_x, arena_y, args.fence_height,
                  args.edge_sigma),
            loss="soft_l1", f_scale=1.0, max_nfev=500, verbose=1)
        joint_initial = np.concatenate((fixed_fit.x, np.zeros(len(observations) * 3)))
        joint_fit = least_squares(
            joint_residual, joint_initial,
            bounds=(np.concatenate((mount_lower, correction_lower)),
                    np.concatenate((mount_upper, correction_upper))),
            args=(observations, measured_poses, arena_x, arena_y, args.fence_height,
                  args.edge_sigma, args.position_sigma, yaw_sigma),
            jac_sparsity=joint_jacobian_sparsity(observations), tr_solver="lsmr",
            loss="soft_l1", f_scale=1.0, max_nfev=500, verbose=1)
        mount = joint_fit.x[:6]
        corrections = joint_fit.x[6:].reshape(-1, 3)
    fitted_poses = measured_poses + corrections
    if args.tags:
        for observation in observations:
            observation["upper_rays"] = rectified_pixel_rays(
                observation["upper_pixels"], fitted_matrix)
            observation["lower_rays"] = rectified_pixel_rays(
                observation["lower_pixels"], fitted_matrix)

    pose_reports = []
    all_distances = []
    for observation, measured, fitted, correction in zip(
            observations, measured_poses, fitted_poses, corrections):
        if args.tags:
            pieces = []
            for corner in tagged_corners(observation):
                first = projected_wall_line(mount, fitted, corner["walls"][0],
                                            corner["edge_height"], fitted_matrix,
                                            arena_x, arena_y)
                second = projected_wall_line(mount, fitted, corner["walls"][1],
                                             corner["edge_height"], fitted_matrix,
                                             arena_x, arena_y)
                intersection = np.cross(first, second)
                predicted = intersection[:2] / intersection[2]
                pieces.append(np.linalg.norm(predicted - corner["pixel"]))
            distances = np.asarray(pieces)
        else:
            _, _, upper_distance = intersect_edge(observation["upper_rays"], args.fence_height,
                                                  mount, fitted, arena_x, arena_y)
            _, _, lower_distance = intersect_edge(observation["lower_rays"], 0.0, mount, fitted,
                                                  arena_x, arena_y)
            distances = np.concatenate((upper_distance, lower_distance))
        all_distances.extend(distances)
        if args.tags and not len(distances):
            error_fields = {
                "corner_count": 0,
                "median_corner_error_px": None,
                "p90_corner_error_px": None,
                "rmse_corner_error_px": None,
                "max_corner_error_px": None,
            }
        elif args.tags:
            error_fields = {
                "corner_count": len(distances),
                "median_corner_error_px": float(np.median(distances)),
                "p90_corner_error_px": float(np.percentile(distances, 90)),
                "rmse_corner_error_px": float(np.sqrt(np.mean(np.square(distances)))),
                "max_corner_error_px": float(np.max(distances)),
            }
        else:
            error_fields = {
            "median_edge_error_m": float(np.median(distances)),
            "p90_edge_error_m": float(np.percentile(distances, 90)),
            "rmse_edge_error_m": float(np.sqrt(np.mean(np.square(distances)))),
            "max_edge_error_m": float(np.max(distances)),
            }
        pose_reports.append({
            "id": observation["id"],
            "measured_pose": {"x_m": measured[0], "y_m": measured[1],
                              "yaw_deg": math.degrees(measured[2])},
            "fitted_pose": {"x_m": fitted[0], "y_m": fitted[1],
                            "yaw_deg": math.degrees(fitted[2])},
            "correction": {"x_m": correction[0], "y_m": correction[1],
                           "yaw_deg": math.degrees(correction[2])},
            **error_fields,
        })
    all_distances = np.asarray(all_distances)
    report = {
        "mount": dict(zip(MOUNT_NAMES, map(float, mount))),
        "rectified_intrinsics": {
            "fx": float(fitted_matrix[0, 0]), "fy": float(fitted_matrix[1, 1]),
            "cx": float(fitted_matrix[0, 2]), "cy": float(fitted_matrix[1, 2]),
            "refined": bool(args.tags and args.refine_rectified_intrinsics),
        },
        "fit": {
            "success": bool(joint_fit.success),
            "message": joint_fit.message,
            "error_unit": "px" if args.tags else "m",
            "median_error": float(np.median(all_distances)),
            "p90_error": float(np.percentile(all_distances, 90)),
            "rmse_error": float(np.sqrt(np.mean(np.square(all_distances)))),
            "max_error": float(np.max(all_distances)),
            "sample_count": int(len(all_distances)),
            "metric": "visible_corner_reprojection" if args.tags else "edge_to_arena",
            "position_prior_sigma_m": args.position_sigma,
            "yaw_prior_sigma_deg": args.yaw_sigma_deg,
        },
        "poses": pose_reports,
    }
    (output_dir / "fit.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    storage = cv2.FileStorage(str(output_dir / "camera_mount.yaml"), cv2.FILE_STORAGE_WRITE)
    try:
        for name, value in zip(MOUNT_NAMES, mount):
            storage.write(name, float(value))
        storage.write("rectified_K", fitted_matrix)
        storage.write("fence_height_m", args.fence_height)
        storage.write("error_metric", report["fit"]["metric"])
        storage.write("error_unit", report["fit"]["error_unit"])
        storage.write("median_error", report["fit"]["median_error"])
        storage.write("p90_error", report["fit"]["p90_error"])
        storage.write("rmse_error", report["fit"]["rmse_error"])
        storage.write("max_error", report["fit"]["max_error"])
    finally:
        storage.release()
    render_diagnostics(output_dir, observations, fitted_poses, mount,
                       fitted_matrix if args.tags else diagnostic_matrix,
                       diagnostic_distortion,
                       arena_x, arena_y, args.fence_height, args.pixel_sigma)

    print("\nFitted camera mount:")
    for name, value in zip(MOUNT_NAMES, mount):
        print("  {:>10s}: {:.6f}".format(name, value))
    if args.tags:
        if args.refine_rectified_intrinsics:
            print("rectified intrinsics: fx={:.3f} fy={:.3f} cx={:.3f} cy={:.3f}".format(
                fitted_matrix[0, 0], fitted_matrix[1, 1],
                fitted_matrix[0, 2], fitted_matrix[1, 2]))
        print("visible-corner error: median={:.2f}px p90={:.2f}px rmse={:.2f}px max={:.2f}px".format(
            report["fit"]["median_error"], report["fit"]["p90_error"],
            report["fit"]["rmse_error"], report["fit"]["max_error"]))
    else:
        print("median edge error: {:.1f} mm; p90: {:.1f} mm".format(
            1000.0 * report["fit"]["median_error"],
            1000.0 * report["fit"]["p90_error"]))
    print("wrote {}".format(output_dir))
    return 0 if joint_fit.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
