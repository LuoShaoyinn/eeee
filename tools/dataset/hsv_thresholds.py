"""Shared HSV threshold configuration for the arena auto-labeling tools."""

import json
from pathlib import Path

import cv2
import numpy as np

DEFAULT_THRESHOLDS = {
    "blue_fence": [[[92, 75, 45], [135, 255, 255]]],
    "white_ground": [[[0, 0, 165], [180, 70, 255]]],
    "red_cube": [[[0, 95, 55], [10, 255, 255]], [[170, 95, 55], [180, 255, 255]]],
    "yellow_cylinder": [[[18, 90, 70], [42, 255, 255]]],
}


def load_thresholds(path: Path) -> dict[str, list[list[list[int]]]]:
    thresholds = {name: [[lower[:], upper[:]] for lower, upper in ranges]
                  for name, ranges in DEFAULT_THRESHOLDS.items()}
    if not path.exists():
        return thresholds
    loaded = json.loads(path.read_text())
    for name, ranges in loaded.items():
        if name not in thresholds or not isinstance(ranges, list):
            continue
        valid = []
        for value in ranges:
            if (isinstance(value, list) and len(value) == 2 and
                    all(isinstance(bound, list) and len(bound) == 3 for bound in value) and
                    all(isinstance(channel, int) and 0 <= channel <= 255
                        for bound in value for channel in bound)):
                valid.append(value)
        if valid:
            thresholds[name] = valid
    return thresholds


def save_thresholds(path: Path, thresholds: dict[str, list[list[list[int]]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(thresholds, indent=2) + "\n")


def make_mask(hsv: np.ndarray, ranges: list[list[list[int]]]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    return mask
