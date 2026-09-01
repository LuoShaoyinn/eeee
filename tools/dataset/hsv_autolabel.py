#!/usr/bin/env python3
"""Create editable semantic masks and YOLO box candidates using HSV thresholds."""

import argparse
from pathlib import Path

import cv2
import numpy as np

SEMANTIC = {"other": 0, "white_ground": 1, "blue_fence": 2}
DETECTION = {"red_cube": 0, "yellow_cylinder": 1, "opponent_robot": 2}


def hsv_range(hsv: np.ndarray, lower: tuple[int, int, int], upper: tuple[int, int, int]) -> np.ndarray:
    return cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))


def clean(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def boxes(mask: np.ndarray, class_id: int, width: int, height: int, min_area: int) -> list[str]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    labels = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        labels.append(f"{class_id} {(x + w / 2) / width:.6f} {(y + h / 2) / height:.6f} "
                      f"{w / width:.6f} {h / height:.6f}")
    return labels


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--masks", type=Path, default=Path("dataset/semantic_masks"))
    parser.add_argument("--labels", type=Path, default=Path("dataset/detection_labels"))
    parser.add_argument("--preview", type=Path, default=Path("dataset/previews"))
    parser.add_argument("--min-object-area", type=int, default=120)
    args = parser.parse_args()
    if args.min_object_area <= 0:
        parser.error("--min-object-area must be positive")

    for directory in (args.masks, args.labels, args.preview):
        directory.mkdir(parents=True, exist_ok=True)

    image_paths = sorted(path for path in args.images.iterdir()
                         if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"skipping unreadable image: {image_path}")
            continue
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        white = clean(hsv_range(hsv, (0, 0, 165), (180, 70, 255)), 5)
        blue = clean(hsv_range(hsv, (92, 75, 45), (135, 255, 255)), 5)
        red = clean(hsv_range(hsv, (0, 95, 55), (10, 255, 255)) |
                    hsv_range(hsv, (170, 95, 55), (180, 255, 255)), 3)
        yellow = clean(hsv_range(hsv, (18, 90, 70), (42, 255, 255)), 3)

        semantic = np.zeros((height, width), dtype=np.uint8)
        semantic[white > 0] = SEMANTIC["white_ground"]
        semantic[blue > 0] = SEMANTIC["blue_fence"]
        stem = image_path.stem
        cv2.imwrite(str(args.masks / f"{stem}.png"), semantic)

        label_lines = boxes(red, DETECTION["red_cube"], width, height, args.min_object_area)
        label_lines += boxes(yellow, DETECTION["yellow_cylinder"], width, height,
                             args.min_object_area)
        (args.labels / f"{stem}.txt").write_text("\n".join(label_lines) +
                                                  ("\n" if label_lines else ""))

        overlay = image.copy()
        overlay[semantic == SEMANTIC["white_ground"]] = (255, 255, 255)
        overlay[semantic == SEMANTIC["blue_fence"]] = (255, 0, 0)
        cv2.imwrite(str(args.preview / f"{stem}.jpg"), cv2.addWeighted(image, .60, overlay, .40, 0))

    print(f"tagged {len(image_paths)} images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
