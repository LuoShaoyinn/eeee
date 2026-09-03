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
    parser.add_argument("--global-confirmations", type=int, default=8)
    parser.add_argument("--visual-pull", type=float, default=.12)
    parser.add_argument("--visual-max-step", type=float, default=.04)
    parser.add_argument("--visual-max-yaw-step-deg", type=float, default=1.5)
    parser.add_argument("--visual-certain-pull", type=float, default=.50)
    parser.add_argument("--visual-certain-max-step", type=float, default=.15)
    parser.add_argument("--visual-certain-max-yaw-step-deg", type=float, default=6.0)
    parser.add_argument("--visual-precise-residual", type=float, default=.010)
    parser.add_argument("--visual-dominant-residual", type=float, default=.030)
    parser.add_argument("--visual-certain-margin", type=float, default=.015)
    parser.add_argument("--visual-min-lower-edge-points", type=int, default=60)
    parser.add_argument("--visual-yaw-reset-max-error-deg", type=float, default=15.0)
    args = parser.parse_args()
    if args.particles < 100 or not 0 < args.visual_pull <= 1 or args.visual_max_step <= 0:
        parser.error("invalid particle count or visual pull limits")
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
    gyro_bias_radps = 0.0
    bias_anchor_yaw = None
    bias_anchor_elapsed = 0.0
    bias_anchor_imu_delta = 0.0
    pending_global = None
    pending_global_count = 0
    confirmed_global = None
    with output_path.open("w") as output:
        for index, row in enumerate(rows):
            now_ns = row["monotonic_ns"]
            dt = min(max((now_ns - previous_ns) / 1e9, 0), .2)
            previous_ns = now_ns
            forward, left = row["wheel"][:2]
            measured_yaw_rate = -math.radians(row["gyro_z_degps"])
            yaw_rate = measured_yaw_rate - gyro_bias_radps
            bias_anchor_elapsed += dt
            bias_anchor_imu_delta += measured_yaw_rate * dt
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
            visual_certain = False
            yaw_reset = False
            visual_reason = "no_measurement"
            alternative_margin = (candidates[1][3] - candidates[0][3]
                                  if len(candidates) > 1 else math.inf)
            if candidates:
                precise = candidates[0][3] <= args.visual_precise_residual
                dominant_edge = (
                    candidates[0][3] <= args.visual_dominant_residual and
                    alternative_margin >= args.visual_certain_margin and
                    row.get("lower_fence_points", 0) >=
                    args.visual_min_lower_edge_points)
                visual_certain = precise or dominant_edge
                visual_partial = (
                    candidates[0][3] <= args.visual_dominant_residual and
                    row.get("lower_fence_points", 0) >=
                    args.visual_min_lower_edge_points)
                comparable = [
                    item for item in candidates
                    if item[3] <= candidates[0][3] + .015
                ]
                maximum_yaw_disagreement = max(
                    abs(float(wrap_angle(item[2] - candidates[0][2])))
                    for item in comparable)
                visual_very_certain = (
                    precise and row.get("lower_fence_points", 0) >=
                    args.visual_min_lower_edge_points and
                    maximum_yaw_disagreement <= math.radians(3))
            else:
                visual_very_certain = False
                visual_partial = False
            if index % 5 == 0 and candidates and candidates[0][3] < args.visual_residual_limit:
                visual_reason = "innovation"
                if visual_certain:
                    visual_accepted = True
                    visual_reason = "certain"
                    confirmed_global = np.asarray(candidates[0][:2])
                    pending_global = None
                    pending_global_count = 0
                elif visual_partial:
                    visual_accepted = True
                    visual_reason = "partial"
                elif innovation < .35:
                    visual_accepted = True
                    visual_reason = "local"
                    pending_global = None
                    pending_global_count = 0
                elif candidates[0][3] < .020:
                    candidate_position = np.asarray(candidates[0][:2])
                    if confirmed_global is not None and np.linalg.norm(
                            candidate_position - confirmed_global) < .25:
                        visual_accepted = True
                        visual_reason = "global_tracking"
                        confirmed_global = candidate_position
                    elif pending_global is not None and np.linalg.norm(
                            candidate_position - pending_global) < .25:
                        visual_reason = "global_pending"
                        pending_global_count += 1
                    else:
                        visual_reason = "global_pending"
                        pending_global = candidate_position
                        pending_global_count = 1
                    if pending_global_count >= args.global_confirmations:
                        visual_accepted = True
                        visual_reason = "global_confirmed"
                        confirmed_global = candidate_position
                        pending_global = None
                        pending_global_count = 0
            if visual_accepted:
                candidates = np.asarray(candidates[:1], dtype=np.float64)
                candidate_weights = np.ones(1)
                current_pose, _, _ = estimate(particles, weights)
                delta = candidates[0, :2] - np.asarray(current_pose[:2])
                if len(candidate0) and len(row.get(
                        "visual_geometry_candidates", [])) > 1:
                    alternatives = np.asarray([
                        item[:2] for item in row["visual_geometry_candidates"]
                        if item[3] <= candidate0[3] + .020 and
                        abs(float(wrap_angle(item[2] - candidate0[2]))) <=
                        math.radians(5)
                    ])
                    if len(alternatives) > 1:
                        covariance = np.cov(alternatives.T)
                        values, vectors = np.linalg.eigh(covariance)
                        major = vectors[:, int(np.argmax(values))]
                        major_sigma = math.sqrt(max(0.0, float(np.max(values))))
                        major_component = float(delta @ major) * major
                        minor_component = delta - major_component
                        major_gain = (0.0 if major_sigma >= .15 else
                                      float(np.clip(.05 / (major_sigma + .01), 0.0, 1.0)))
                        delta = minor_component + major_gain * major_component
                delta_length = float(np.linalg.norm(delta))
                pull = args.visual_certain_pull if visual_certain else args.visual_pull
                max_step = (args.visual_certain_max_step if visual_certain
                            else args.visual_max_step)
                pull_length = min(max_step, pull * delta_length)
                if delta_length > 1e-9:
                    delta *= pull_length / delta_length
                yaw_delta = float(wrap_angle(candidates[0, 2] - current_pose[2]))
                yaw_reset = (visual_very_certain and abs(yaw_delta) <=
                             math.radians(args.visual_yaw_reset_max_error_deg))
                yaw_step_limit = math.radians(
                    args.visual_certain_max_yaw_step_deg if visual_certain
                    else args.visual_max_yaw_step_deg)
                yaw_step = (yaw_delta if yaw_reset else
                            float(np.clip(pull * yaw_delta,
                                          -yaw_step_limit, yaw_step_limit)))
                particles[:, :2] += delta
                particles[:, 2] = wrap_angle(particles[:, 2] + yaw_step)
                particles[:, 0] = np.clip(particles[:, 0], 0, 3)
                particles[:, 1] = np.clip(particles[:, 1], 0, 1.985)
                if visual_very_certain:
                    visual_yaw = float(candidates[0, 2])
                    if bias_anchor_yaw is None:
                        bias_anchor_yaw = visual_yaw
                        bias_anchor_elapsed = 0.0
                        bias_anchor_imu_delta = 0.0
                    elif bias_anchor_elapsed >= 1.0:
                        visual_delta = float(wrap_angle(visual_yaw - bias_anchor_yaw))
                        observed_bias = ((bias_anchor_imu_delta - visual_delta) /
                                         bias_anchor_elapsed)
                        observed_bias = float(np.clip(observed_bias,
                                                      -math.radians(5),
                                                      math.radians(5)))
                        gyro_bias_radps += .15 * (observed_bias - gyro_bias_radps)
                        bias_anchor_yaw = visual_yaw
                        bias_anchor_elapsed = 0.0
                        bias_anchor_imu_delta = 0.0
                # Weight around the bounded intermediate target, not the full
                # visual jump. Repeated good frames complete the correction.
                measurement = np.array([current_pose[0] + delta[0],
                                        current_pose[1] + delta[1],
                                        wrap_angle(current_pose[2] + yaw_step)])
                likelihood = np.zeros(count)
                yaw_sigma = math.radians(args.visual_yaw_sigma_deg)
                for candidate_weight in candidate_weights:
                    distance2 = ((particles[:, 0] - measurement[0]) ** 2 +
                                 (particles[:, 1] - measurement[1]) ** 2)
                    yaw_error = wrap_angle(particles[:, 2] - measurement[2])
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
                "visual_certain": bool(visual_accepted and visual_certain),
                "imu_yaw_reset": bool(visual_accepted and yaw_reset),
                "gyro_bias_degps": math.degrees(gyro_bias_radps),
                "visual_reason": visual_reason,
                "visual_candidate": candidate0,
                "visual_innovation_m": innovation if math.isfinite(innovation) else None,
                "visual_speed_used": False,
            }, separators=(",", ":")) + "\n")
    print("wrote {} frames, accepted {} visual updates: {}".format(
        len(rows), accepted_updates, output_path))


if __name__ == "__main__":
    main()
