#!/usr/bin/env python3
"""Interactively tune HSV ranges against one arena image."""

import argparse
from pathlib import Path

import cv2

from hsv_thresholds import load_thresholds, make_mask, save_thresholds

RANGE_KEYS = {
    ord("1"): ("blue_fence", 0),
    ord("2"): ("white_ground", 0),
    ord("3"): ("home_black", 0),
    ord("4"): ("red_cube", 0),
    ord("5"): ("red_cube", 1),
    ord("6"): ("yellow_cylinder", 0),
}
COLORS = {
    "white_ground": (255, 255, 255),
    "blue_fence": (255, 0, 0),
    "home_black": (255, 0, 255),
    "red_cube": (0, 0, 255),
    "yellow_cylinder": (0, 255, 255),
}


def noop(_value: int) -> None:
    pass


def first_image(directory: Path) -> Path:
    images = sorted(path for path in directory.iterdir()
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        raise FileNotFoundError(f"no images found in {directory}")
    return images[0]


def set_trackbars(window: str, lower: list[int], upper: list[int]) -> None:
    for name, value in zip(("LH", "LS", "LV", "UH", "US", "UV"), lower + upper):
        cv2.setTrackbarPos(name, window, value)


def read_trackbars(window: str) -> tuple[list[int], list[int]]:
    values = [cv2.getTrackbarPos(name, window)
              for name in ("LH", "LS", "LV", "UH", "US", "UV")]
    return values[:3], values[3:]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="image to tune against (default: first dataset image)")
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    args = parser.parse_args()

    image_path = args.image or first_image(args.images)
    image = cv2.imread(str(image_path))
    if image is None:
        parser.error(f"cannot read {image_path}")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    thresholds = load_thresholds(args.thresholds)
    selected, range_index = "blue_fence", 0

    controls, preview = "HSV controls", "HSV preview"
    cv2.namedWindow(controls, cv2.WINDOW_NORMAL)
    cv2.namedWindow(preview, cv2.WINDOW_NORMAL)
    for name, maximum in (("LH", 180), ("LS", 255), ("LV", 255),
                          ("UH", 180), ("US", 255), ("UV", 255)):
        cv2.createTrackbar(name, controls, 0, maximum, noop)
    set_trackbars(controls, *thresholds[selected][range_index])
    print("keys: 1 blue fence, 2 white ground, 3 black home, 4 red low-H, "
          "5 red high-H, 6 yellow; "
          "s save profile, q quit")

    while True:
        lower, upper = read_trackbars(controls)
        thresholds[selected][range_index] = [lower, upper]
        selected_mask = make_mask(hsv, thresholds[selected])
        canvas = image.copy()
        overlay = image.copy()
        overlay[selected_mask > 0] = COLORS[selected]
        canvas = cv2.addWeighted(canvas, .55, overlay, .45, 0)
        cv2.putText(canvas, f"{image_path.name} | {selected}[{range_index}]", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 0), 3)
        cv2.putText(canvas, f"{image_path.name} | {selected}[{range_index}]", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 255, 0), 1)
        cv2.imshow(preview, canvas)
        key = cv2.waitKey(16) & 0xFF
        if key == 255:
            continue
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            save_thresholds(args.thresholds, thresholds)
            print(f"saved {args.thresholds}")
        elif key in RANGE_KEYS:
            selected, range_index = RANGE_KEYS[key]
            set_trackbars(controls, *thresholds[selected][range_index])

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
