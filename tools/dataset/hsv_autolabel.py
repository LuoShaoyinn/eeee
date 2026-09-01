#!/usr/bin/env python3
"""Create editable semantic masks and YOLO box candidates using HSV thresholds."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from hsv_thresholds import load_thresholds, make_mask

SEMANTIC = {"other": 0, "blue_fence": 1, "white_ground": 2, "home_black": 3}
DETECTION = {"opponent_robot": 0, "red_cube": 1, "yellow_cylinder": 2}


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
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"),
                        help="JSON threshold profile written by hsv_tune.py")
    parser.add_argument("--classes", nargs="+", choices=("blue_fence", "white_ground", "home_black",
                                                            "red_cube", "yellow_cylinder"),
                        default=["blue_fence"],
                        help="classes to auto-label (default: blue_fence only)")
    parser.add_argument("--min-object-area", type=int, default=120)
    args = parser.parse_args()
    if args.min_object_area <= 0:
        parser.error("--min-object-area must be positive")

    for directory in (args.masks, args.labels, args.preview):
        directory.mkdir(parents=True, exist_ok=True)
    thresholds = load_thresholds(args.thresholds)
    enabled = set(args.classes)

    image_paths = sorted(path for path in args.images.iterdir()
                         if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"skipping unreadable image: {image_path}")
            continue
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        white = clean(make_mask(hsv, thresholds["white_ground"]), 5) \
            if "white_ground" in enabled else np.zeros((height, width), np.uint8)
        blue = clean(make_mask(hsv, thresholds["blue_fence"]), 5) \
            if "blue_fence" in enabled else np.zeros((height, width), np.uint8)
        home = clean(make_mask(hsv, thresholds["home_black"]), 3) \
            if "home_black" in enabled else np.zeros((height, width), np.uint8)
        # A home mark is dark floor below a blue fence, not every image shadow.
        home[~np.maximum.accumulate(blue > 0, axis=0)] = 0
        red = clean(make_mask(hsv, thresholds["red_cube"]), 3) \
            if "red_cube" in enabled else np.zeros((height, width), np.uint8)
        yellow = clean(make_mask(hsv, thresholds["yellow_cylinder"]), 3) \
            if "yellow_cylinder" in enabled else np.zeros((height, width), np.uint8)

        semantic = np.zeros((height, width), dtype=np.uint8)
        semantic[blue > 0] = SEMANTIC["blue_fence"]
        semantic[white > 0] = SEMANTIC["white_ground"]
        semantic[home > 0] = SEMANTIC["home_black"]
        stem = image_path.stem
        cv2.imwrite(str(args.masks / f"{stem}.png"), semantic)

        label_lines = boxes(red, DETECTION["red_cube"], width, height, args.min_object_area)
        label_lines += boxes(yellow, DETECTION["yellow_cylinder"], width, height,
                             args.min_object_area)
        (args.labels / f"{stem}.txt").write_text("\n".join(label_lines) +
                                                  ("\n" if label_lines else ""))

        overlay = image.copy()
        overlay[semantic == SEMANTIC["white_ground"]] = (255, 255, 255)
        overlay[semantic == SEMANTIC["blue_fence"]] = (255, 0, 255)
        overlay[semantic == SEMANTIC["home_black"]] = (30, 30, 30)
        cv2.imwrite(str(args.preview / f"{stem}.jpg"), cv2.addWeighted(image, .60, overlay, .40, 0))

    print(f"tagged {len(image_paths)} images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
