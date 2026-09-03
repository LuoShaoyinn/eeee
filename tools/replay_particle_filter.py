#!/usr/bin/env python3
"""Offline multimodal PF using wheel translation, relative IMU yaw, and visual poses."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def wrap_angle(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def estimate(particles, weights):
    x_bin = np.clip((particles[:, 0] / .10).astype(int), 0, 29)
    y_bin = np.clip((particles[:, 1] / .10).astype(int), 0, 19)
    yaw_bin = ((particles[:, 2] + np.pi) / math.radians(10)).astype(int) % 36
    bins = (x_bin * 20 + y_bin) * 36 + yaw_bin
    mode = int(np.argmax(np.bincount(bins, weights=weights, minlength=30 * 20 * 36)))
    mode_yaw = ((mode % 36) + .5) * math.radians(10) - np.pi
    spatial = mode // 36
    mode_y = ((spatial % 20) + .5) * .10
    mode_x = ((spatial // 20) + .5) * .10
    selected = ((particles[:, 0] - mode_x) ** 2 + (particles[:, 1] - mode_y) ** 2 < .20 ** 2) & \
               (np.abs(wrap_angle(particles[:, 2] - mode_yaw)) < math.radians(12))
    local_weights = weights[selected]
    local_weights /= local_weights.sum()
    local = particles[selected]
    x_m = float(np.sum(local_weights * local[:, 0]))
    y_m = float(np.sum(local_weights * local[:, 1]))
    yaw = math.atan2(float(np.sum(local_weights * np.sin(local[:, 2]))),
                     float(np.sum(local_weights * np.cos(local[:, 2]))))
    dx = local[:, 0] - x_m
    dy = local[:, 1] - y_m
    position_sigma = math.sqrt(float(np.sum(local_weights * (dx * dx + dy * dy))))
    yaw_sigma = math.sqrt(float(np.sum(local_weights * wrap_angle(local[:, 2] - yaw) ** 2)))
    return [x_m, y_m, yaw], position_sigma, yaw_sigma


def systematic_resample(rng, particles, weights):
    count = len(weights)
    positions = (rng.random() + np.arange(count)) / count
    indices = np.searchsorted(np.cumsum(weights), positions)
    return particles[indices].copy(), np.full(count, 1.0 / count)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--particles", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--visual-residual-limit", type=float, default=.045)
    parser.add_argument("--visual-position-sigma", type=float, default=.14)
    parser.add_argument("--visual-yaw-sigma-deg", type=float, default=8)
    parser.add_argument("--visual-injection", type=float, default=.12)
    parser.add_argument("--global-confirmations", type=int, default=8)
    args = parser.parse_args()
    if args.particles < 100 or not 0 <= args.visual_injection < 1:
        parser.error("need at least 100 particles and injection in [0,1)")
    output_path = args.output or args.telemetry.with_name("offline-pf.jsonl")
    rows = [json.loads(line) for line in args.telemetry.open()]
    if not rows:
        raise RuntimeError("empty telemetry log")
    rng = np.random.default_rng(args.seed)
    count = args.particles
    particles = np.column_stack([
        rng.normal(.1, .05, count),
        rng.normal(.1, .05, count),
        rng.normal(0, math.radians(5), count),
    ])
    particles[:, 0] = np.clip(particles[:, 0], 0, 3)
    particles[:, 1] = np.clip(particles[:, 1], 0, 1.985)
    weights = np.full(count, 1.0 / count)
    previous_ns = rows[0]["monotonic_ns"]
    accepted_updates = 0
    previous_pose = [.1, .1, 0]
    pending_global = None
    pending_global_count = 0
    with output_path.open("w") as output:
        for index, row in enumerate(rows):
            now_ns = row["monotonic_ns"]
            dt = min(max((now_ns - previous_ns) / 1e9, 0), .2)
            previous_ns = now_ns
            forward, left = row["wheel"][:2]
            yaw_rate = -math.radians(row["gyro_z_degps"])
            midpoint_yaw = particles[:, 2] + .5 * yaw_rate * dt
            distance = math.hypot(forward, left) * dt
            translation_noise = .0005 + .10 * distance
            particles[:, 0] += ((np.cos(midpoint_yaw) * forward -
                                 np.sin(midpoint_yaw) * left) * dt +
                                rng.normal(0, translation_noise, count))
            particles[:, 1] += ((np.sin(midpoint_yaw) * forward +
                                 np.cos(midpoint_yaw) * left) * dt +
                                rng.normal(0, translation_noise, count))
            particles[:, 2] = wrap_angle(
                particles[:, 2] + yaw_rate * dt +
                rng.normal(0, .0003 + .03 * abs(yaw_rate * dt), count))
            particles[:, 0] = np.clip(particles[:, 0], 0, 3)
            particles[:, 1] = np.clip(particles[:, 1], 0, 1.985)

            candidates = row.get("visual_geometry_candidates", [])
            candidate0 = candidates[0] if candidates else None
            innovation = (math.hypot(candidates[0][0] - previous_pose[0],
                                     candidates[0][1] - previous_pose[1])
                          if candidates else math.inf)
            visual_accepted = False
            visual_reason = "no_measurement"
            if index % 5 == 0 and candidates and candidates[0][3] < args.visual_residual_limit:
                visual_reason = "innovation"
                if innovation < .35:
                    visual_accepted = True
                    visual_reason = "local"
                    pending_global = None
                    pending_global_count = 0
                elif candidates[0][3] < .020:
                    visual_reason = "global_pending"
                    candidate_position = np.asarray(candidates[0][:2])
                    if pending_global is not None and np.linalg.norm(
                            candidate_position - pending_global) < .25:
                        pending_global_count += 1
                    else:
                        pending_global = candidate_position
                        pending_global_count = 1
                    if pending_global_count >= args.global_confirmations:
                        visual_accepted = True
                        visual_reason = "global_confirmed"
                        pending_global = None
                        pending_global_count = 0
            if visual_accepted:
                candidates = np.asarray(candidates[:1], dtype=np.float64)
                candidate_weights = np.ones(1)
                injection_fraction = args.visual_injection if innovation >= .35 else .02
                injected = int(injection_fraction * count)
                choices = np.zeros(injected, dtype=int)
                replace = np.argpartition(weights, injected)[:injected]
                particles[replace, 0] = rng.normal(candidates[choices, 0], .12)
                particles[replace, 1] = rng.normal(candidates[choices, 1], .12)
                particles[replace, 2] = rng.normal(
                    candidates[choices, 2], math.radians(6))
                particles[replace, 0] = np.clip(particles[replace, 0], 0, 3)
                particles[replace, 1] = np.clip(particles[replace, 1], 0, 1.985)
                particles[replace, 2] = wrap_angle(particles[replace, 2])
                weights[replace] = 1.0 / count
                likelihood = np.zeros(count)
                yaw_sigma = math.radians(args.visual_yaw_sigma_deg)
                for candidate_weight, candidate in zip(candidate_weights, candidates):
                    distance2 = ((particles[:, 0] - candidate[0]) ** 2 +
                                 (particles[:, 1] - candidate[1]) ** 2)
                    yaw_error = wrap_angle(particles[:, 2] - candidate[2])
                    likelihood += candidate_weight * np.exp(
                        -.5 * distance2 / args.visual_position_sigma ** 2 -
                        .5 * yaw_error ** 2 / yaw_sigma ** 2)
                weights *= 1e-6 + likelihood
                weights /= weights.sum()
                accepted_updates += 1
                if 1.0 / np.sum(weights * weights) < .55 * count:
                    particles, weights = systematic_resample(rng, particles, weights)
                    particles[:, 0] += rng.normal(0, .008, count)
                    particles[:, 1] += rng.normal(0, .008, count)
                    particles[:, 2] = wrap_angle(
                        particles[:, 2] + rng.normal(0, math.radians(.4), count))
                    particles[:, 0] = np.clip(particles[:, 0], 0, 3)
                    particles[:, 1] = np.clip(particles[:, 1], 0, 1.985)

            pose, position_sigma, yaw_sigma = estimate(particles, weights)
            previous_pose = pose
            output.write(json.dumps({
                "frame_index": row["frame_index"],
                "monotonic_ns": now_ns,
                "pose": pose,
                "position_sigma_m": position_sigma,
                "yaw_sigma_rad": yaw_sigma,
                "effective_particles": float(1.0 / np.sum(weights * weights)),
                "visual_accepted": bool(visual_accepted),
                "visual_reason": visual_reason,
                "visual_candidate": candidate0,
                "visual_innovation_m": innovation if math.isfinite(innovation) else None,
                "visual_speed_used": False,
            }, separators=(",", ":")) + "\n")
    print("wrote {} frames, accepted {} visual updates: {}".format(
        len(rows), accepted_updates, output_path))


if __name__ == "__main__":
    main()
