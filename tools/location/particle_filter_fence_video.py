#!/usr/bin/env python3
"""Replay robot telemetry with image-space fence likelihood in a particle filter."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

import match_fence_video_image_space as matching
import project_fence_video_bev as fence


def wrap_angle(values):
    return (values + math.pi) % (2.0 * math.pi) - math.pi


def systematic_resample(weights, generator):
    count = len(weights)
    positions = (generator.random() + np.arange(count)) / count
    return np.searchsorted(np.cumsum(weights), positions, side="right")


def estimate(particles, weights):
    x = float(weights @ particles[:, 0])
    y = float(weights @ particles[:, 1])
    yaw = math.atan2(float(weights @ np.sin(particles[:, 2])),
                     float(weights @ np.cos(particles[:, 2])))
    distances = np.linalg.norm(particles[:, :2] - (x, y), axis=1)
    order = np.argsort(distances)
    radius95 = float(distances[order[np.searchsorted(np.cumsum(weights[order]), .95)]])
    return np.array((x, y, yaw)), radius95


def propagate(particles, velocity, dt, generator):
    forward, left, yaw_rate = velocity
    travel = math.hypot(forward, left) * dt
    translation_sigma = .002 + .06 * travel
    yaw_sigma = math.radians(.15) + .08 * abs(yaw_rate) * dt
    noisy_forward = forward * dt + generator.normal(0, translation_sigma, len(particles))
    noisy_left = left * dt + generator.normal(0, translation_sigma, len(particles))
    cosine, sine = np.cos(particles[:, 2]), np.sin(particles[:, 2])
    particles[:, 0] += cosine * noisy_forward - sine * noisy_left
    particles[:, 1] += sine * noisy_forward + cosine * noisy_left
    particles[:, 2] = wrap_angle(
        particles[:, 2] + yaw_rate * dt + generator.normal(0, yaw_sigma, len(particles)))
    particles[:, 0] = np.clip(particles[:, 0], 0, matching.ARENA_X)
    particles[:, 1] = np.clip(particles[:, 1], 0, matching.ARENA_Y)


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
    # Logged IMU axes are +x forward, +y right, +z down. Only integrate the
    # relative robot-frame yaw rate; do not use the IMU's absolute yaw angle.
    imu_yaw_rate = -math.radians(float(record["gyro_z_degps"]))
    velocity[2] = (.8 * imu_yaw_rate + .2 * visual_velocity[2]
                   if plausible_visual else imu_yaw_rate)
    return velocity


def image_scores(matrix, arena, upper_map, lower_map, upper, lower,
                 fence_height, particles):
    matcher = matching.ImageMatcher(matrix, arena, upper_map, lower_map,
                                    fence_height, np.zeros(3), 1e9, 1e9,
                                    upper, lower)
    return np.asarray([matcher.image_cost(particle) for particle in particles])


def inject_global_proposals(particles, weights, matrix, arena, upper_map, lower_map,
                            upper, lower, fence_height, fraction, generator):
    xs = np.linspace(0.0, matching.ARENA_X, 13)
    ys = np.linspace(0.0, matching.ARENA_Y, 9)
    yaws = np.linspace(-math.pi, math.pi, 16, endpoint=False)
    grid = np.asarray([(x, y, yaw) for x in xs for y in ys for yaw in yaws])
    scores = image_scores(matrix, arena, upper_map, lower_map, upper, lower,
                          fence_height, grid)
    seeds = grid[np.argsort(scores)[:16]]
    count = max(1, round(len(particles) * fraction))
    selected = seeds[generator.integers(0, len(seeds), count)]
    proposals = selected + np.column_stack((
        generator.normal(0, .12, count), generator.normal(0, .12, count),
        generator.normal(0, math.radians(8.0), count)))
    proposals[:, 0] = np.clip(proposals[:, 0], 0, matching.ARENA_X)
    proposals[:, 1] = np.clip(proposals[:, 1], 0, matching.ARENA_Y)
    proposals[:, 2] = wrap_angle(proposals[:, 2])
    replace = np.argsort(weights)[:count]
    particles[replace] = proposals
    weights *= 1.0 - fraction
    weights[replace] = fraction / count
    weights /= np.sum(weights)
    return float(scores.min())


def draw_bev(particles, weights, pose, trajectory, radius95, timestamp):
    scale, margin = 260, 60
    width = round(matching.ARENA_X * scale) + 2 * margin
    height = round(matching.ARENA_Y * scale) + 2 * margin
    canvas = np.full((height, width, 3), 245, np.uint8)

    def point(x, y):
        return round(margin + x * scale), round(height - margin - y * scale)

    maximum = max(float(np.max(weights)), 1e-12)
    for particle, weight in zip(particles, weights):
        intensity = int(np.clip(40 + 215 * math.sqrt(weight / maximum), 0, 255))
        cv2.circle(canvas, point(particle[0], particle[1]), 2,
                   (255 - intensity // 2, 80, intensity), -1)
    cv2.rectangle(canvas, point(0, matching.ARENA_Y),
                  point(matching.ARENA_X, 0), (20, 20, 20), 3)
    if len(trajectory) > 1:
        path = np.asarray([point(item[0], item[1]) for item in trajectory], np.int32)
        cv2.polylines(canvas, [path], False, (255, 255, 255), 5, cv2.LINE_AA)
        cv2.polylines(canvas, [path], False, (25, 25, 25), 2, cv2.LINE_AA)
    origin = point(pose[0], pose[1])
    tip = point(pose[0] + .20 * math.cos(pose[2]),
                pose[1] + .20 * math.sin(pose[2]))
    cv2.circle(canvas, origin, max(2, round(radius95 * scale)), (0, 100, 255), 2,
               cv2.LINE_AA)
    cv2.arrowedLine(canvas, origin, tip, (0, 0, 230), 4, cv2.LINE_AA, tipLength=.3)
    cv2.putText(canvas, "particle filter t={:.1f}s r95={:.2f}m".format(timestamp, radius95),
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .64, (20, 20, 20), 2,
                cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--projective-fit", type=Path, required=True)
    parser.add_argument("--hsv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--particles", type=int, default=1200)
    parser.add_argument("--sample-fps", type=float, default=1.0)
    parser.add_argument("--initial-x", type=float, default=.10)
    parser.add_argument("--initial-y", type=float, default=.10)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--fence-height", type=float, default=.254)
    parser.add_argument("--image-sigma", type=float, default=10.0)
    parser.add_argument("--global-proposal-interval", type=int, default=10,
                        help="inject image-derived global proposals every N sampled frames")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.particles < 100:
        parser.error("--particles must be at least 100")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    overlay_dir, bev_dir = args.output_dir / "overlays", args.output_dir / "bev"
    overlay_dir.mkdir(exist_ok=True)
    bev_dir.mkdir(exist_ok=True)
    matrix = np.asarray(json.loads(args.projective_fit.read_text())["projection_matrix"],
                        np.float64)
    ranges = fence.load_blue_ranges(args.hsv)
    telemetry = matching.load_telemetry(args.telemetry)
    arena = matching.arena_samples()
    generator = np.random.default_rng(args.seed)
    particles = np.column_stack((
        generator.normal(args.initial_x, .05, args.particles),
        generator.normal(args.initial_y, .05, args.particles),
        generator.normal(math.radians(args.initial_yaw), math.radians(5), args.particles)))
    particles[:, 0] = np.clip(particles[:, 0], 0, matching.ARENA_X)
    particles[:, 1] = np.clip(particles[:, 1], 0, matching.ARENA_Y)
    particles[:, 2] = wrap_angle(particles[:, 2])
    weights = np.full(args.particles, 1.0 / args.particles)

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("cannot open {}".format(args.video))
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    next_sample = 0.0
    frame_index = sample_index = last_sample_frame = 0
    trajectory, reports = [], []
    writer = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index + 1e-6 < next_sample:
            frame_index += 1
            continue
        for index in range(last_sample_frame + 1, frame_index + 1):
            record, previous = telemetry.get(index), telemetry.get(index - 1)
            if record and previous:
                dt = np.clip((record["monotonic_ns"] - previous["monotonic_ns"]) * 1e-9,
                             0.0, .25)
                propagate(particles, corrected_body_velocity(record), float(dt), generator)

        mask = fence.fence_mask(frame, ranges, 500)
        upper, lower = fence.boundary_pixels(mask, 8)
        upper_map = matching.edge_distance_map(mask.shape, upper)
        lower_map = matching.edge_distance_map(mask.shape, lower)
        proposal_score = None
        if (args.global_proposal_interval > 0 and sample_index > 0 and
                sample_index % args.global_proposal_interval == 0):
            proposal_score = inject_global_proposals(
                particles, weights, matrix, arena, upper_map, lower_map, upper, lower,
                args.fence_height, .20, generator)
        scores = image_scores(matrix, arena, upper_map, lower_map, upper, lower,
                              args.fence_height, particles)
        finite = scores < 20.0
        updated = np.count_nonzero(finite) >= max(30, args.particles // 20)
        if updated:
            log_likelihood = -.5 * np.square(scores / args.image_sigma)
            log_weights = np.log(np.maximum(weights, 1e-300)) + log_likelihood
            log_weights -= np.max(log_weights)
            weights = np.exp(log_weights)
            weights /= np.sum(weights)
        pose, radius95 = estimate(particles, weights)
        ess = 1.0 / float(weights @ weights)
        trajectory.append(pose.copy())
        timestamp = frame_index / source_fps
        score = matching.ImageMatcher(matrix, arena, upper_map, lower_map,
                                      args.fence_height, pose, 1e9, 1e9,
                                      upper, lower).image_cost(pose)
        overlay = matching.draw_overlay(frame, upper, lower, matrix, arena, pose,
                                        args.fence_height, score)
        bev = draw_bev(particles, weights, pose, trajectory, radius95, timestamp)
        name = "frame-{:04d}".format(sample_index)
        cv2.imwrite(str(overlay_dir / (name + ".jpg")), overlay,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(bev_dir / (name + ".png")), bev)
        bev_width = round(bev.shape[1] * 720 / bev.shape[0])
        combined = np.hstack((overlay, cv2.resize(bev, (bev_width, 720))))
        if writer is None:
            writer = cv2.VideoWriter(str(args.output_dir / "particle-filter.avi"),
                                     cv2.VideoWriter_fourcc(*"MJPG"), args.sample_fps,
                                     (combined.shape[1], combined.shape[0]))
        writer.write(combined)
        reports.append({"frame": frame_index, "time_s": timestamp,
                        "pose": pose.tolist(), "radius95_m": radius95,
                        "image_score_px": score, "effective_particles": ess,
                        "measurement_updated": bool(updated),
                        "global_proposal_score_px": proposal_score})
        print("frame {:4d}: score={:5.2f}px pose=({:.3f}, {:.3f}, {:.1f}deg) r95={:.3f}m ess={:.0f}".format(
            frame_index, score, pose[0], pose[1], math.degrees(pose[2]), radius95, ess))
        if updated and ess < .55 * args.particles:
            particles = particles[systematic_resample(weights, generator)]
            weights.fill(1.0 / args.particles)
        last_sample_frame = frame_index
        sample_index += 1
        next_sample += source_fps / args.sample_fps
        frame_index += 1

    capture.release()
    if writer is not None:
        writer.release()
    (args.output_dir / "trajectory.json").write_text(json.dumps(reports, indent=2) + "\n")
    print("wrote {}".format(args.output_dir))


if __name__ == "__main__":
    main()
