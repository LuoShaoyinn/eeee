#!/usr/bin/env python3
"""Generate visual evidence for the offline trajectory cases."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

from trajectory_planner_sim import Pose, draw_plan, fence_clearance, plan_collection


def main() -> int:
    output_dir = Path("artifacts/trajectory-cases")
    output_dir.mkdir(parents=True, exist_ok=True)
    cases = {
        "open": ((1.5, 1.0, 0), (2.2, 1.1)),
        "left_fence": ((1.5, 1.0, 0), (.18, 1.0)),
        "right_fence": ((1.5, 1.0, 0), (2.82, 1.0)),
        "bottom_fence": ((1.5, 1.0, 0), (1.5, .18)),
        "top_fence": ((1.5, 1.0, 0), (1.5, 1.805)),
        "upper_right_corner": ((1.5, 1.0, 0), (2.82, 1.75)),
        "invalid_start": ((.20, .20, 0), (1.0, 1.0)),
    }
    report = {}
    images = []
    for name, (start_values, target_values) in cases.items():
        start = Pose(start_values[0], start_values[1], math.radians(start_values[2]))
        target = np.asarray(target_values, dtype=np.float64)
        path = plan_collection(start, target, .10)
        image_path = output_dir / f"{name}.png"
        draw_plan(path, target, image_path, .10)
        image = cv2.imread(str(image_path))
        cv2.putText(image, name.replace("_", " "), (50, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (20, 20, 20), 2)
        cv2.imwrite(str(image_path), image)
        images.append(cv2.resize(image, (600, 436)))
        report[name] = {
            "expected": "infeasible" if name == "invalid_start" else "feasible",
            "feasible": path is not None,
            "yaw_deg": None if path is None else round(math.degrees(path[-1].yaw), 2),
            "minimum_waypoint_clearance_m": None if path is None else
                round(min(fence_clearance(pose) for pose in path), 4),
            "waypoints": [] if path is None else
                [[round(p.x, 4), round(p.y, 4), round(math.degrees(p.yaw), 2)]
                 for p in path],
        }
    width = max(image.shape[1] for image in images)
    blank = np.full_like(images[0], 245)
    while len(images) % 2:
        images.append(blank)
    montage = np.vstack([np.hstack(images[index:index + 2])
                         for index in range(0, len(images), 2)])
    cv2.imwrite(str(output_dir / "montage.png"), montage)
    (output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    failures = [name for name, result in report.items()
                if result["feasible"] != (result["expected"] == "feasible")]
    print(f"generated={len(report)} failures={len(failures)} output={output_dir}")
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
