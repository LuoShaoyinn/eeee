#!/usr/bin/env python3
"""Localize a rectified arena video by scoring projected walls in image space."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

import project_fence_video_bev as fence


ARENA_X = 3.0
ARENA_Y = 1.985


def wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def load_telemetry(path):
    if path is None:
        return {}
    records = {}
    with path.open(encoding="utf-8") as source:
        for line in source:
            record = json.loads(line)
            records[int(record["frame_index"])] = record
    return records


def arena_samples(step=0.04):
    corners = np.array(((0.0, 0.0), (ARENA_X, 0.0),
                        (ARENA_X, ARENA_Y), (0.0, ARENA_Y)), np.float64)
    points = []
    for first, second in zip(corners, np.roll(corners, -1, axis=0)):
        count = max(2, round(np.linalg.norm(second - first) / step) + 1)
        fraction = np.linspace(0.0, 1.0, count)
        points.append(np.outer(1.0 - fraction, first) + np.outer(fraction, second))
    return np.vstack(points)


def edge_distance_map(shape, points):
    image = np.full(shape, 255, np.uint8)
    if len(points):
        coordinates = np.rint(points).astype(np.int32)
        valid = ((coordinates[:, 0] >= 0) & (coordinates[:, 0] < shape[1]) &
                 (coordinates[:, 1] >= 0) & (coordinates[:, 1] < shape[0]))
        coordinates = coordinates[valid]
        image[coordinates[:, 1], coordinates[:, 0]] = 0
        image = cv2.dilate(255 - image, np.ones((3, 3), np.uint8))
        image = 255 - image
    return cv2.distanceTransform(image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def world_to_robot(points, pose, height):
    delta = points - pose[:2]
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((cosine * delta[:, 0] + sine * delta[:, 1],
                            -sine * delta[:, 0] + cosine * delta[:, 1],
                            np.full(len(points), height), np.ones(len(points))))


def project_arena(matrix, points, pose, height, width, image_height):
    homogeneous = world_to_robot(points, pose, height) @ matrix.T
    valid = homogeneous[:, 2] > 1e-7
    pixels = homogeneous[valid, :2] / homogeneous[valid, 2:3]
    valid_pixels = ((pixels[:, 0] >= 1) & (pixels[:, 0] < width - 1) &
                    (pixels[:, 1] >= 1) & (pixels[:, 1] < image_height - 1))
    return pixels[valid_pixels]


def sample_map(distance_map, pixels):
    if len(pixels) < 20:
        return np.empty(0, np.float64)
    coordinates = np.rint(pixels).astype(np.int32)
    return distance_map[coordinates[:, 1], coordinates[:, 0]]


class ImageMatcher:
    def __init__(self, matrix, arena, upper_map, lower_map, fence_height,
                 predicted_pose, position_sigma, yaw_sigma,
                 observed_upper=None, observed_lower=None):
        self.matrix = matrix
        self.arena = arena
        self.upper_map = upper_map
        self.lower_map = lower_map
        self.fence_height = fence_height
        self.predicted_pose = predicted_pose
        self.position_sigma = position_sigma
        self.yaw_sigma = yaw_sigma
        self.observed_upper = observed_upper
        self.observed_lower = observed_lower
        self.height, self.width = lower_map.shape

    def image_cost(self, pose):
        upper = project_arena(self.matrix, self.arena, pose, self.fence_height,
                              self.width, self.height)
        lower = project_arena(self.matrix, self.arena, pose, 0.0,
                              self.width, self.height)
        distances = []
        for distance_map, pixels, observed in (
                (self.upper_map, upper, self.observed_upper),
                (self.lower_map, lower, self.observed_lower)):
            values = sample_map(distance_map, pixels)
            if not len(values):
                return 100.0
            # Obstacles may hide a wall, so use a capped robust image-space loss.
            values = np.minimum(values, 20.0)
            distances.append(np.mean(np.minimum(values * values, 100.0)))
            if observed is not None and len(observed):
                reverse = cKDTree(pixels).query(observed[::8])[0]
                distances.append(np.mean(np.minimum(reverse * reverse, 100.0)))
        return math.sqrt(sum(distances) / len(distances))

    def cost(self, pose):
        pose = np.asarray(pose, np.float64)
        image = self.image_cost(pose)
        position_delta = np.linalg.norm(pose[:2] - self.predicted_pose[:2])
        yaw_delta = wrap_angle(pose[2] - self.predicted_pose[2])
        prior = 0.35 * ((position_delta / self.position_sigma) ** 2 +
                        (yaw_delta / self.yaw_sigma) ** 2)
        return image + prior


def propagate(pose, velocity, dt):
    forward, left, yaw_rate = velocity
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return np.array((np.clip(pose[0] + dt * (cosine * forward - sine * left), 0, ARENA_X),
                     np.clip(pose[1] + dt * (sine * forward + cosine * left), 0, ARENA_Y),
                     wrap_angle(pose[2] + dt * yaw_rate)), np.float64)


def corrected_body_velocity(record):
    wheel = np.asarray(record["wheel"], np.float64)
    visual = record["visual"]
    visual_velocity = np.asarray(visual["velocity"], np.float64)
    plausible_visual = (visual["valid"] and
                        np.linalg.norm(visual_velocity[:2]) < .8 and
                        abs(visual_velocity[2]) < 4.0)
    velocity = wheel.copy()
    if plausible_visual:
        velocity[:2] = .7 * wheel[:2] + .3 * visual_velocity[:2]
    imu_yaw_rate = -math.radians(float(record["gyro_z_degps"]))
    velocity[2] = (.8 * imu_yaw_rate + .2 * visual_velocity[2]
                   if plausible_visual else imu_yaw_rate)
    return velocity


def search_pose(matcher, position_range, yaw_range, grid_size):
    predicted = matcher.predicted_pose
    x_values = np.linspace(max(0.0, predicted[0] - position_range),
                           min(ARENA_X, predicted[0] + position_range), grid_size)
    y_values = np.linspace(max(0.0, predicted[1] - position_range),
                           min(ARENA_Y, predicted[1] + position_range), grid_size)
    yaw_values = predicted[2] + np.linspace(-yaw_range, yaw_range, grid_size)
    heat = np.empty((len(y_values), len(x_values)), np.float64)
    best = (math.inf, predicted.copy())
    for row, y in enumerate(y_values):
        for column, x in enumerate(x_values):
            scores = [(matcher.cost((x, y, yaw)), yaw) for yaw in yaw_values]
            score, yaw = min(scores, key=lambda item: item[0])
            heat[row, column] = score
            if score < best[0]:
                best = (score, np.array((x, y, wrap_angle(yaw))))
    bounds = ((max(0.0, predicted[0] - position_range),
               min(ARENA_X, predicted[0] + position_range)),
              (max(0.0, predicted[1] - position_range),
               min(ARENA_Y, predicted[1] + position_range)),
              (best[1][2] - yaw_range / grid_size, best[1][2] + yaw_range / grid_size))
    refined = minimize(matcher.cost, best[1], method="Powell", bounds=bounds,
                       options={"maxiter": 80, "xtol": 1e-4, "ftol": 1e-3})
    pose = np.asarray(refined.x, np.float64)
    pose[2] = wrap_angle(pose[2])
    return pose, matcher.image_cost(pose), heat, x_values, y_values


def draw_overlay(frame, upper, lower, matrix, arena, pose, fence_height, score):
    output = frame.copy()
    for points, colour in ((upper, (255, 255, 0)), (lower, (0, 165, 255))):
        for point in np.rint(points).astype(np.int32):
            cv2.circle(output, tuple(point), 1, colour, -1)
    for height, colour in ((fence_height, (255, 0, 255)), (0.0, (0, 255, 0))):
        pixels = project_arena(matrix, arena, pose, height, frame.shape[1], frame.shape[0])
        for point in np.rint(pixels).astype(np.int32):
            cv2.circle(output, tuple(point), 2, colour, -1)
    cv2.putText(output, "projected map score={:.2f}px".format(score), (24, 34),
                cv2.FONT_HERSHEY_SIMPLEX, .70, (255, 255, 255), 2, cv2.LINE_AA)
    return output


def draw_bev(heat, x_values, y_values, pose, trajectory, timestamp):
    scale, margin = 260, 60
    width, height = round(ARENA_X * scale) + 2 * margin, round(ARENA_Y * scale) + 2 * margin
    canvas = np.full((height, width, 3), 245, np.uint8)
    normalized = cv2.normalize(heat, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    coloured = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
    x0, x1 = round(margin + x_values[0] * scale), round(margin + x_values[-1] * scale)
    y0, y1 = round(height - margin - y_values[-1] * scale), round(height - margin - y_values[0] * scale)
    canvas[y0:y1 + 1, x0:x1 + 1] = cv2.resize(coloured, (x1 - x0 + 1, y1 - y0 + 1))

    def point(x, y):
        return round(margin + x * scale), round(height - margin - y * scale)

    cv2.rectangle(canvas, point(0, ARENA_Y), point(ARENA_X, 0), (20, 20, 20), 3)
    if len(trajectory) > 1:
        path = np.asarray([point(item[0], item[1]) for item in trajectory], np.int32)
        cv2.polylines(canvas, [path], False, (255, 255, 255), 5, cv2.LINE_AA)
        cv2.polylines(canvas, [path], False, (30, 30, 30), 2, cv2.LINE_AA)
    origin = point(pose[0], pose[1])
    tip = point(pose[0] + .20 * math.cos(pose[2]), pose[1] + .20 * math.sin(pose[2]))
    cv2.arrowedLine(canvas, origin, tip, (0, 0, 255), 4, cv2.LINE_AA, tipLength=.3)
    cv2.putText(canvas, "image-space matching t={:.1f}s".format(timestamp), (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, .66, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(canvas, "blue/red: better/worse local score", (20, 52),
                cv2.FONT_HERSHEY_SIMPLEX, .48, (30, 30, 30), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--projective-fit", type=Path, required=True)
    parser.add_argument("--hsv", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-fps", type=float, default=1.0)
    parser.add_argument("--initial-x", type=float, default=.10)
    parser.add_argument("--initial-y", type=float, default=.10)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--fence-height", type=float, default=.254)
    parser.add_argument("--position-range", type=float, default=.20)
    parser.add_argument("--yaw-range-deg", type=float, default=10.0)
    parser.add_argument("--grid-size", type=int, default=7)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    overlays = args.output_dir / "overlays"
    bevs = args.output_dir / "bev"
    overlays.mkdir(exist_ok=True)
    bevs.mkdir(exist_ok=True)
    matrix = np.asarray(json.loads(args.projective_fit.read_text())["projection_matrix"],
                        np.float64)
    ranges = fence.load_blue_ranges(args.hsv)
    telemetry = load_telemetry(args.telemetry)
    arena = arena_samples()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("cannot open {}".format(args.video))
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    interval = source_fps / args.sample_fps
    pose = np.array((args.initial_x, args.initial_y,
                     math.radians(args.initial_yaw)), np.float64)
    trajectory, reports = [pose.copy()], []
    next_sample = 0.0
    last_sample_frame = 0
    frame_index = sample_index = 0
    writer = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index + 1e-6 < next_sample:
            frame_index += 1
            continue
        for index in range(last_sample_frame + 1, frame_index + 1):
            record = telemetry.get(index)
            previous = telemetry.get(index - 1)
            if record and previous:
                dt = (record["monotonic_ns"] - previous["monotonic_ns"]) * 1e-9
                pose = propagate(pose, corrected_body_velocity(record),
                                 min(max(dt, 0.0), .25))
        predicted = pose.copy()
        mask = fence.fence_mask(frame, ranges, 500)
        upper, lower = fence.boundary_pixels(mask, 8)
        upper_map = edge_distance_map(mask.shape, upper)
        lower_map = edge_distance_map(mask.shape, lower)
        matcher = ImageMatcher(matrix, arena, upper_map, lower_map, args.fence_height,
                               predicted, args.position_range,
                               math.radians(args.yaw_range_deg), upper, lower)
        pose, score, heat, xs, ys = search_pose(
            matcher, args.position_range, math.radians(args.yaw_range_deg), args.grid_size)
        recovered = False
        if score >= 20.0:
            # A fast turn can put the odometry prediction outside the local basin.
            # Search all headings, but retain a strict translational continuity gate.
            recovery = ImageMatcher(matrix, arena, upper_map, lower_map,
                                    args.fence_height, predicted, .40,
                                    math.radians(90.0), upper, lower)
            candidate, candidate_score, candidate_heat, candidate_xs, candidate_ys = search_pose(
                recovery, max(.40, 2.0 * args.position_range), math.pi, 13)
            displacement = np.linalg.norm(candidate[:2] - predicted[:2])
            if candidate_score < score and displacement <= .45:
                pose, score = candidate, candidate_score
                heat, xs, ys = candidate_heat, candidate_xs, candidate_ys
                recovered = True
        matched = score < 20.0
        if not matched:
            pose = predicted
        trajectory.append(pose.copy())
        timestamp = frame_index / source_fps
        overlay = draw_overlay(frame, upper, lower, matrix, arena, pose,
                               args.fence_height, score)
        bev = draw_bev(heat, xs, ys, pose, trajectory, timestamp)
        name = "frame-{:04d}".format(sample_index)
        cv2.imwrite(str(overlays / (name + ".jpg")), overlay,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(bevs / (name + ".png")), bev)
        target_height = 720
        bev_width = round(bev.shape[1] * target_height / bev.shape[0])
        combined = np.hstack((overlay, cv2.resize(bev, (bev_width, target_height))))
        if writer is None:
            writer = cv2.VideoWriter(str(args.output_dir / "image-space-match.avi"),
                                     cv2.VideoWriter_fourcc(*"MJPG"), args.sample_fps,
                                     (combined.shape[1], combined.shape[0]))
        writer.write(combined)
        reports.append({"frame": frame_index, "time_s": timestamp,
                        "predicted_pose": predicted.tolist(), "pose": pose.tolist(),
                        "image_score_px": score, "matched": matched,
                        "global_recovery": recovered})
        print("frame {:4d}: score={:5.2f}px pose=({:.3f}, {:.3f}, {:.1f}deg){}".format(
            frame_index, score, pose[0], pose[1], math.degrees(pose[2]),
            " recovered" if recovered else ""))
        last_sample_frame = frame_index
        sample_index += 1
        next_sample += interval
        frame_index += 1
    capture.release()
    if writer is not None:
        writer.release()
    (args.output_dir / "matches.json").write_text(json.dumps(reports, indent=2) + "\n")
    print("wrote {}".format(args.output_dir))


if __name__ == "__main__":
    main()
