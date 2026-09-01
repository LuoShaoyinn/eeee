#!/usr/bin/env python3
"""Fit up to three straight segments to the upper and lower blue-fence edges."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from hsv_thresholds import load_thresholds, make_mask


def retain_fence_components(mask: np.ndarray, minimum_area_ratio: float = 0.05) -> np.ndarray:
    """Discard small blue regions such as reflections and blue robot details."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return mask
    largest_area = int(np.max(stats[1:, cv2.CC_STAT_AREA]))
    keep = np.flatnonzero(stats[:, cv2.CC_STAT_AREA] >= largest_area * minimum_area_ratio)
    keep = keep[keep != 0]
    return np.where(np.isin(labels, keep), 255, 0).astype(np.uint8)


def column_edges(mask: np.ndarray, min_pixels: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    blue = mask > 0
    above = np.zeros_like(blue)
    below = np.zeros_like(blue)
    above[1:] = blue[:-1]
    below[:-1] = blue[1:]
    upper_boundary = blue & ~above
    lower_boundary = blue & ~below
    upper_transition = np.zeros_like(mask, dtype=np.uint8)
    lower_transition = np.zeros_like(mask, dtype=np.uint8)
    xs, upper, lower = [], [], []
    for x in range(mask.shape[1]):
        upper_y = np.flatnonzero(upper_boundary[:, x])
        lower_y = np.flatnonzero(lower_boundary[:, x])
        if np.count_nonzero(blue[:, x]) >= min_pixels and len(upper_y) and len(lower_y):
            xs.append(x)
            upper.append(upper_y[0])
            lower.append(lower_y[-1])
            upper_transition[upper_y[0], x] = 255
            lower_transition[lower_y[-1], x] = 255
    return (np.asarray(xs, float), np.asarray(upper, float),
            np.asarray(lower, float), upper_transition, lower_transition)


def hough_panel_lines(edge: np.ndarray, maximum: int) -> list[np.ndarray]:
    """Find the longest distinct straight pieces of the segmented lower contour."""
    detected = cv2.HoughLinesP(edge, 1, np.pi / 180, threshold=50,
                                minLineLength=max(80, edge.shape[1] // 12), maxLineGap=40)
    if detected is None:
        return []
    candidates: list[tuple[float, float, float, float, np.ndarray]] = []
    for x0, y0, x1, y1 in detected.reshape(-1, 4):
        if x0 == x1:
            continue
        start, end = sorted((float(x0), float(x1)))
        slope = (float(y1) - float(y0)) / (float(x1) - float(x0))
        intercept = float(y0) - slope * float(x0)
        candidates.append((end - start, start, end, slope, np.asarray([slope, intercept])))

    selected: list[tuple[float, float, float, float, np.ndarray]] = []
    for candidate in sorted(candidates, reverse=True, key=lambda value: value[0]):
        _, start, end, slope, line = candidate
        duplicate = False
        for _, other_start, other_end, other_slope, other_line in selected:
            overlap = max(0.0, min(end, other_end) - max(start, other_start))
            if (overlap >= 0.5 * min(end - start, other_end - other_start) and
                    abs(slope - other_slope) < 0.03 and
                    abs(np.polyval(line, (start + end) / 2) -
                        np.polyval(other_line, (start + end) / 2)) < 8):
                duplicate = True
                break
        if not duplicate:
            selected.append(candidate)
        if len(selected) == maximum:
            break
    return [line for _, _, _, _, line in sorted(selected, key=lambda value: value[1])]


def draw_hough_panels(image: np.ndarray, lines: list[np.ndarray],
                      color: tuple[int, int, int]) -> None:
    """Join long contour segments and bridge occluded parts with straight panels."""
    if not lines:
        return
    joins: list[float] = []
    for index, (left, right) in enumerate(zip(lines, lines[1:])):
        denominator = left[0] - right[0]
        if abs(denominator) < 1e-6:
            continue
        joins.append(float((right[1] - left[1]) / denominator))

    boundaries = [0.0, *joins, float(image.shape[1] - 1)]
    if len(boundaries) != len(lines) + 1:
        boundaries = np.linspace(0, image.shape[1] - 1, len(lines) + 1).tolist()
    for index, coefficients in enumerate(lines):
        start, end = boundaries[index], boundaries[index + 1]
        points_x = np.arange(round(start), round(end) + 1, dtype=float)
        points_y = np.polyval(coefficients, points_x)
        points = np.column_stack((points_x, points_y)).round().astype(np.int32)
        cv2.polylines(image, [points], False, color, 3, cv2.LINE_AA)
    for join, left in zip(joins, lines):
        cv2.circle(image, (round(join), round(np.polyval(left, join))), 4, color, -1)


def process(image_path: Path, output: Path, blue_ranges: list[list[list[int]]],
            white_ranges: list[list[list[int]]], black_ranges: list[list[list[int]]], maximum: int,
            min_pixels: int, kernel_output: Path | None) -> bool:
    image = cv2.imread(str(image_path))
    if image is None:
        return False
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = make_mask(hsv, blue_ranges)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    blue = retain_fence_components(blue)
    white = make_mask(hsv, white_ranges)
    black = make_mask(hsv, black_ranges)
    floor = white | black
    x, upper, lower, upper_transition, lower_transition = column_edges(blue, min_pixels)
    overlay = image.copy()
    overlay[floor > 0] = (0, 220, 220)  # yellow: candidate white/black floor
    overlay[blue > 0] = (255, 0, 255)   # magenta: blue-fence HSV support
    result = cv2.addWeighted(image, .65, overlay, .35, 0)
    if kernel_output is not None:
        kernel = result.copy()
        kernel[upper_transition > 0] = (0, 255, 0)  # green: segmented upper contour
        kernel[lower_transition > 0] = (0, 0, 255)  # red: segmented lower contour
        kernel_output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(kernel_output / f"{image_path.stem}.jpg"), kernel)
    if len(x) >= 20:
        draw_hough_panels(result, hough_panel_lines(upper_transition, maximum), (0, 255, 0))
        draw_hough_panels(result, hough_panel_lines(lower_transition, maximum), (0, 165, 255))
    cv2.imwrite(str(output / f"{image_path.stem}.jpg"), result)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--output", type=Path, default=Path("dataset/fence_edges"))
    parser.add_argument("--thresholds", type=Path, default=Path("dataset/hsv_thresholds.json"))
    parser.add_argument("--segments", type=int, default=3, choices=(1, 2, 3))
    parser.add_argument("--min-column-pixels", type=int, default=8)
    parser.add_argument("--kernel-output", type=Path,
                        help="optional output directory for the segmented blue lower-edge diagnostic")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    thresholds = load_thresholds(args.thresholds)
    images = sorted(path for path in args.images.iterdir()
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    count = sum(process(path, args.output, thresholds["blue_fence"], thresholds["white_ground"],
                        thresholds["home_black"], args.segments,
                        args.min_column_pixels, args.kernel_output) for path in images)
    print(f"wrote {count} fence overlays to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
