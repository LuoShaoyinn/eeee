#!/usr/bin/env python3
"""Offline footprint-aware trajectory prototype; never sends robot commands."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

ARENA = (3.0, 1.985)
BODY_CORNERS = np.array([[-0.28, -0.15], [0.02, -0.15],
                         [0.02, 0.05], [-0.28, 0.05]], dtype=np.float64)
FRONT_MIDPOINT = np.array([0.02, -0.05], dtype=np.float64)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Target:
    track_id: int
    x: float
    y: float
    confidence: float = 1.0


class TargetLock:
    """Mission-level target ownership, independent of detector flicker."""

    def __init__(self, loss_timeout_s: float = 0.75,
                 terminal_loss_timeout_s: float = 1.5,
                 reacquire_radius_m: float = 0.25) -> None:
        self.loss_timeout_s = loss_timeout_s
        self.terminal_loss_timeout_s = terminal_loss_timeout_s
        self.reacquire_radius_m = reacquire_radius_m
        self.target: Target | None = None
        self.last_seen_s = 0.0

    def update(self, detections: list[Target], now_s: float,
               terminal_approach: bool = False) -> Target | None:
        if self.target is None:
            if detections:
                self.target = max(detections, key=lambda item: item.confidence)
                self.last_seen_s = now_s
            return self.target

        same_track = next((item for item in detections
                           if item.track_id == self.target.track_id), None)
        if same_track is None:
            same_track = next((item for item in detections
                               if math.hypot(item.x - self.target.x,
                                             item.y - self.target.y) <=
                               self.reacquire_radius_m), None)
        if same_track is not None:
            self.target = Target(self.target.track_id, same_track.x, same_track.y,
                                 same_track.confidence)
            self.last_seen_s = now_s
        elif now_s - self.last_seen_s > (self.terminal_loss_timeout_s
                                         if terminal_approach
                                         else self.loss_timeout_s):
            self.target = None
        return self.target

    def complete_or_abandon(self) -> None:
        self.target = None


def wrap(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def rotation(yaw: float) -> np.ndarray:
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


def corners(pose: Pose) -> np.ndarray:
    return BODY_CORNERS @ rotation(pose.yaw).T + (pose.x, pose.y)


def safe(pose: Pose, margin: float) -> bool:
    points = corners(pose)
    return bool(np.all(points[:, 0] >= margin) and
                np.all(points[:, 0] <= ARENA[0] - margin) and
                np.all(points[:, 1] >= margin) and
                np.all(points[:, 1] <= ARENA[1] - margin))


def fence_clearance(pose: Pose) -> float:
    points = corners(pose)
    return float(min(np.min(points[:, 0]), ARENA[0] - np.max(points[:, 0]),
                     np.min(points[:, 1]), ARENA[1] - np.max(points[:, 1])))


def interpolate(a: Pose, b: Pose, fraction: float) -> Pose:
    return Pose(a.x + fraction * (b.x - a.x),
                a.y + fraction * (b.y - a.y),
                a.yaw + fraction * wrap(b.yaw - a.yaw))


def segment_safe(a: Pose, b: Pose, margin: float) -> bool:
    count = max(1, math.ceil(math.hypot(b.x - a.x, b.y - a.y) / 0.02),
                math.ceil(abs(wrap(b.yaw - a.yaw)) / math.radians(4)))
    return all(safe(interpolate(a, b, index / count), margin)
               for index in range(count + 1))


def path_safe(path: list[Pose], margin: float) -> bool:
    return all(segment_safe(first, second, margin)
               for first, second in zip(path, path[1:]))


def plan_collection(start: Pose, target: np.ndarray, margin: float) -> list[Pose] | None:
    nominal = math.atan2(target[1] - start.y, target[0] - start.x)
    candidates: list[tuple[float, list[Pose]]] = []
    for offset in np.linspace(-math.pi, math.pi, 145):
        yaw = wrap(nominal + float(offset))
        rotate = rotation(yaw)
        terminal_xy = target - rotate @ FRONT_MIDPOINT
        terminal = Pose(*terminal_xy, yaw)
        staging_xy = terminal_xy - rotate @ np.array([0.30, 0.0])
        staging = Pose(*staging_xy, yaw)
        if not (safe(staging, margin) and safe(terminal, margin)):
            continue
        center = Pose(ARENA[0] / 2, ARENA[1] / 2, start.yaw)
        route_options = [
            [start, staging, terminal],
            [start, center, Pose(center.x, center.y, yaw), staging, terminal],
            [start, Pose(staging.x, start.y, start.yaw),
             Pose(staging.x, start.y, yaw), staging, terminal],
            [start, Pose(start.x, staging.y, start.yaw),
             Pose(start.x, staging.y, yaw), staging, terminal],
        ]
        for path in route_options:
            if not path_safe(path, margin):
                continue
            distance = sum(math.hypot(b.x - a.x, b.y - a.y)
                           for a, b in zip(path, path[1:]))
            yaw_motion = sum(abs(wrap(b.yaw - a.yaw))
                             for a, b in zip(path, path[1:]))
            # Near a fence, approaching from the inward-facing side is almost
            # uniquely feasible. Penalize low-clearance terminal headings much
            # more strongly than deviation from the line of sight.
            clearance = min(fence_clearance(staging), fence_clearance(terminal))
            robustness_cost = .08 / max(clearance - margin + .01, .01)
            cost = (distance + 0.20 * yaw_motion + 0.02 * abs(offset) +
                    robustness_cost)
            candidates.append((cost, path))
    return min(candidates, default=(0.0, None), key=lambda item: item[0])[1]


def draw_plan(path: list[Pose] | None, target: np.ndarray, output: Path,
              margin: float) -> None:
    scale = 360
    pad = 35
    image = np.full((round(ARENA[1] * scale) + 2 * pad,
                     round(ARENA[0] * scale) + 2 * pad, 3), 245, np.uint8)

    def pixel(point: np.ndarray | tuple[float, float]) -> tuple[int, int]:
        return (round(pad + point[0] * scale),
                round(image.shape[0] - pad - point[1] * scale))

    cv2.rectangle(image, pixel((0, ARENA[1])), pixel((ARENA[0], 0)), (40, 40, 40), 3)
    cv2.rectangle(image, pixel((margin, ARENA[1] - margin)),
                  pixel((ARENA[0] - margin, margin)), (180, 180, 180), 1)
    cv2.rectangle(image, pixel((0, .30)), pixel((.20, 0)), (80, 180, 80), 2)
    cv2.circle(image, pixel(target), 7, (0, 80, 230), -1)
    if path:
        samples: list[Pose] = []
        for first, second in zip(path, path[1:]):
            count = max(2, math.ceil(math.hypot(second.x - first.x,
                                               second.y - first.y) / .03))
            samples.extend(interpolate(first, second, index / count)
                           for index in range(count))
        samples.append(path[-1])
        cv2.polylines(image, [np.array([pixel((p.x, p.y)) for p in samples])],
                      False, (200, 80, 20), 2)
        for index, pose in enumerate(samples):
            if index % 5 and index != len(samples) - 1:
                continue
            polygon = np.array([pixel(point) for point in corners(pose)])
            cv2.polylines(image, [polygon], True, (120, 120, 120), 1)
        for pose in path:
            polygon = np.array([pixel(point) for point in corners(pose)])
            cv2.polylines(image, [polygon], True, (20, 20, 200), 2)
            cv2.circle(image, pixel((pose.x, pose.y)), 4, (20, 20, 20), -1)
    else:
        cv2.putText(image, "NO FEASIBLE PATH", (pad + 20, pad + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 220), 2)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"),
                        default=(1.5, 1.0, 0.0))
    parser.add_argument("--target", nargs=2, type=float, metavar=("X", "Y"),
                        default=(2.4, .25))
    parser.add_argument("--margin", type=float, default=.10)
    parser.add_argument("--output", type=Path,
                        default=Path("artifacts/trajectory-plan.png"))
    args = parser.parse_args()

    start = Pose(args.start[0], args.start[1], math.radians(args.start[2]))
    target = np.asarray(args.target, dtype=np.float64)
    path = plan_collection(start, target, args.margin)
    draw_plan(path, target, args.output, args.margin)
    if path is None:
        print(f"infeasible: {args.output}")
        return 2
    terminal = path[-1]
    relative = rotation(terminal.yaw).T @ (target - (terminal.x, terminal.y))
    print(f"output={args.output} yaw={math.degrees(terminal.yaw):.1f}deg "
          f"contact=({relative[0]:.3f},{relative[1]:.3f})m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
