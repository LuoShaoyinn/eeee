#!/usr/bin/env python3
"""Rectify measured arena poses, initialize fence lines, and review them in a GUI."""

import argparse
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np


UPPER = "upper"
LOWER = "lower"
COLOURS = {UPPER: (255, 255, 0), LOWER: (0, 165, 255)}


def atomic_write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def load_calibration(path: Path, width: int, height: int):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open calibration {}".format(path))
    try:
        camera_matrix = storage.getNode("K").mat()
        distortion = storage.getNode("D").mat()
        rectified_matrix = storage.getNode("rectified_K").mat()
        calibration_width = int(storage.getNode("image_width").real())
        calibration_height = int(storage.getNode("image_height").real())
    finally:
        storage.release()
    if (camera_matrix is None or distortion is None or rectified_matrix is None or
            (calibration_width, calibration_height) != (width, height)):
        raise ValueError("calibration does not match the session images")
    maps = cv2.fisheye.initUndistortRectifyMap(
        camera_matrix, distortion, np.eye(3), rectified_matrix,
        (width, height), cv2.CV_16SC2)
    return maps, rectified_matrix


def load_blue_ranges(path: Path):
    ranges = json.loads(path.read_text(encoding="utf-8")).get("blue_fence")
    if not ranges:
        raise ValueError("{} has no blue_fence ranges".format(path))
    return [(np.asarray(low, np.uint8), np.asarray(high, np.uint8)) for low, high in ranges]


def rectified_median(session_dir: Path, pose: dict, maps) -> np.ndarray:
    frames = []
    for item in pose["images"]:
        raw = cv2.imread(str(session_dir / item["path"]), cv2.IMREAD_COLOR)
        if raw is None:
            raise ValueError("cannot read {}".format(session_dir / item["path"]))
        frames.append(cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR))
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def largest_blue_mask(image: np.ndarray, ranges) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape[:2], np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    count, labels, statistics, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return np.zeros_like(mask)
    largest = 1 + int(np.argmax(statistics[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest, 255, 0).astype(np.uint8)


def boundary_points(mask: np.ndarray, edge: str, margin: int = 3) -> np.ndarray:
    points = []
    height, width = mask.shape
    for x in range(margin, width - margin, 2):
        ys = np.flatnonzero(mask[:, x])
        if ys.size == 0:
            continue
        y = int(ys[0] if edge == UPPER else ys[-1])
        if margin < y < height - margin:
            points.append((float(x), float(y)))
    return np.asarray(points, np.float64)


def line_distances(points: np.ndarray, first: np.ndarray, second: np.ndarray) -> np.ndarray:
    direction = second - first
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return np.full(len(points), np.inf)
    return np.abs(direction[0] * (first[1] - points[:, 1]) -
                  (first[0] - points[:, 0]) * direction[1]) / length


def refined_segment(points: np.ndarray) -> list[list[float]]:
    vx, vy, x0, y0 = cv2.fitLine(points.astype(np.float32), cv2.DIST_L2, 0, .01, .01).ravel()
    direction = np.array([vx, vy], np.float64)
    centre = np.array([x0, y0], np.float64)
    coordinate = (points - centre) @ direction
    low, high = np.percentile(coordinate, (2.0, 98.0))
    endpoints = np.vstack((centre + low * direction, centre + high * direction))
    if endpoints[0, 0] > endpoints[1, 0]:
        endpoints = endpoints[::-1]
    return endpoints.tolist()


def ransac_segments(points: np.ndarray, seed: int, max_lines: int = 4,
                    threshold_px: float = 3.0, min_points: int = 20,
                    min_span_px: float = 90.0) -> list[list[list[float]]]:
    if len(points) < min_points:
        return []
    generator = np.random.default_rng(seed)
    remaining = points.copy()
    segments = []
    for _ in range(max_lines):
        if len(remaining) < min_points:
            break
        best = None
        best_score = -1.0
        for _ in range(600):
            indices = generator.choice(len(remaining), 2, replace=False)
            first, second = remaining[indices]
            if np.linalg.norm(second - first) < min_span_px * .35:
                continue
            inliers = line_distances(remaining, first, second) <= threshold_px
            selected = remaining[inliers]
            if len(selected) < min_points:
                continue
            span = float(np.ptp(selected[:, 0]))
            score = len(selected) * min(span / min_span_px, 2.5)
            if span >= min_span_px and score > best_score:
                best, best_score = inliers, score
        if best is None:
            break
        selected = remaining[best]
        segments.append(refined_segment(selected))
        remaining = remaining[~best]
    segments.sort(key=lambda segment: min(segment[0][0], segment[1][0]))
    return segments


def segment_angle(segment) -> float:
    first, second = np.asarray(segment, np.float64)
    return math.atan2(second[1] - first[1], second[0] - first[0])


def segment_y(segment, x: float) -> float:
    first, second = np.asarray(segment, np.float64)
    if abs(second[0] - first[0]) < 1e-6:
        return float((first[1] + second[1]) * .5)
    return float(first[1] + (x - first[0]) * (second[1] - first[1]) / (second[0] - first[0]))


def clean_segments(segments):
    # Keep the longer representative when RANSAC fits the same physical edge twice.
    by_length = sorted(segments, key=lambda item: abs(item[1][0] - item[0][0]), reverse=True)
    kept = []
    for candidate in by_length:
        x0, x1 = sorted((candidate[0][0], candidate[1][0]))
        duplicate = False
        for previous in kept:
            p0, p1 = sorted((previous[0][0], previous[1][0]))
            overlap_low, overlap_high = max(x0, p0), min(x1, p1)
            if overlap_high <= overlap_low:
                continue
            angle = abs(segment_angle(candidate) - segment_angle(previous))
            angle = min(angle, math.pi - angle)
            middle = (overlap_low + overlap_high) * .5
            if angle < math.radians(4.0) and abs(segment_y(candidate, middle) -
                                                  segment_y(previous, middle)) < 10.0:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)

    # A spurious diagonal can combine support from several real adjoining walls.
    # Reject it when at least three shorter, differently angled fits cover its span.
    result = []
    for candidate in kept:
        x0, x1 = sorted((candidate[0][0], candidate[1][0]))
        span = x1 - x0
        alternatives = []
        for other in kept:
            if other is candidate:
                continue
            o0, o1 = sorted((other[0][0], other[1][0]))
            angle = abs(segment_angle(candidate) - segment_angle(other))
            angle = min(angle, math.pi - angle)
            if angle > math.radians(8.0) and min(x1, o1) > max(x0, o0):
                alternatives.append((max(x0, o0), min(x1, o1)))
        if len(alternatives) >= 3:
            intervals = sorted(alternatives)
            covered = 0.0
            start, end = intervals[0]
            for low, high in intervals[1:]:
                if low <= end:
                    end = max(end, high)
                else:
                    covered += end - start
                    start, end = low, high
            covered += end - start
            if span > 0 and covered / span > .65:
                continue
        result.append(candidate)
    return sorted(result, key=lambda item: (item[0][0] + item[1][0]) * .5)


def line_intersection(first, second):
    a = np.cross(np.array([first[0][0], first[0][1], 1.0]),
                 np.array([first[1][0], first[1][1], 1.0]))
    b = np.cross(np.array([second[0][0], second[0][1], 1.0]),
                 np.array([second[1][0], second[1][1], 1.0]))
    point = np.cross(a, b)
    if abs(point[2]) < 1e-8:
        return None
    return (point[:2] / point[2]).tolist()


def segments_to_points(segments, width: int, height: int):
    segments = clean_segments(segments)
    if not segments:
        return []
    first_x = max(0.0, min(point[0] for point in segments[0]))
    last_x = min(float(width - 1), max(point[0] for point in segments[-1]))
    points = [[first_x, segment_y(segments[0], first_x)]]
    for left, right in zip(segments, segments[1:]):
        intersection = line_intersection(left, right)
        left_mid = (left[0][0] + left[1][0]) * .5
        right_mid = (right[0][0] + right[1][0]) * .5
        if (intersection is None or not left_mid - 120 <= intersection[0] <= right_mid + 120 or
                not -80 <= intersection[1] <= height + 80):
            x = (max(point[0] for point in left) + min(point[0] for point in right)) * .5
            intersection = [x, (segment_y(left, x) + segment_y(right, x)) * .5]
        points.append(intersection)
    points.append([last_x, segment_y(segments[-1], last_x)])
    points.sort(key=lambda point: point[0])
    return points


def make_auto_points(mask: np.ndarray, seed: int) -> dict:
    height, width = mask.shape
    upper_segments = ransac_segments(boundary_points(mask, UPPER), seed * 2)
    lower_segments = ransac_segments(boundary_points(mask, LOWER), seed * 2 + 1)
    return {
        UPPER: segments_to_points(upper_segments, width, height),
        LOWER: segments_to_points(lower_segments, width, height),
    }


def nearest_point(points, point):
    if not points:
        return None
    distances = [math.hypot(candidate[0] - point[0], candidate[1] - point[1])
                 for candidate in points]
    index = int(np.argmin(distances))
    return distances[index], index


class Reviewer:
    def __init__(self, tags: dict, tags_path: Path, session_dir: Path):
        self.tags = tags
        self.tags_path = tags_path
        self.session_dir = session_dir
        self.entries = tags["poses"]
        self.index = next((index for index, entry in enumerate(self.entries)
                           if not entry.get("reviewed")), 0)
        self.edge = UPPER
        self.dragging = None
        self.adding = False
        self.add_start = None
        self.window = "Arena fence lines"

    def current(self):
        return self.entries[self.index]

    def save(self):
        atomic_write_json(self.tags_path, self.tags)

    def draw(self):
        entry = self.current()
        image = cv2.imread(str(self.session_dir / entry["rectified_image"]))
        if image is None:
            raise RuntimeError("cannot read {}".format(entry["rectified_image"]))
        for edge in (UPPER, LOWER):
            colour = COLOURS[edge]
            thickness = 4 if edge == self.edge else 2
            points = np.rint(entry[edge]).astype(np.int32)
            if len(points) >= 2:
                cv2.polylines(image, [points], False, colour, thickness, cv2.LINE_AA)
            for point in points:
                cv2.circle(image, tuple(point), 7, colour, -1, cv2.LINE_AA)
                cv2.circle(image, tuple(point), 7, (20, 20, 20), 1, cv2.LINE_AA)
        if self.add_start is not None:
            cv2.circle(image, tuple(map(round, self.add_start)), 9, COLOURS[self.edge], 2)
        return image

    def print_status(self):
        entry = self.current()
        state = "reviewed" if entry.get("reviewed") else "unreviewed"
        print("pose {:02d} ({}/{}) {} selected={} x={:.3f} y={:.3f} yaw={:.1f}".format(
            entry["id"], self.index + 1, len(self.entries), state, self.edge,
            entry["x_m"], entry["y_m"], entry["yaw_deg"]))

    def mouse(self, event, x, y, _flags, _parameter):
        point = [float(x), float(y)]
        points = self.current()[self.edge]
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.adding:
                points.append(point)
                points.sort(key=lambda candidate: candidate[0])
                self.adding = False
                self.current()["reviewed"] = False
                return
            nearest = nearest_point(points, point)
            if nearest is not None and nearest[0] <= 18.0:
                self.dragging = nearest[1]
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging is not None:
            points[self.dragging] = point
            self.current()["reviewed"] = False
        elif event == cv2.EVENT_LBUTTONUP:
            if self.dragging is not None:
                points.sort(key=lambda candidate: candidate[0])
            self.dragging = None
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            nearest = nearest_point(points, point)
            if nearest is not None and nearest[0] <= 18.0:
                del points[nearest[1]]
                self.current()["reviewed"] = False

    def run(self):
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 1280, 720)
        cv2.setMouseCallback(self.window, self.mouse)
        print("U/L select | drag corner | A then click to add | right-click delete | "
              "R restore auto | Space accept | P previous | Q save/quit")
        self.print_status()
        try:
            while True:
                cv2.imshow(self.window, self.draw())
                key = cv2.waitKey(20) & 0xFF
                if key == 255:
                    continue
                if key in (ord("q"), 27):
                    self.save()
                    break
                if key == ord("u"):
                    self.edge = UPPER
                    self.print_status()
                elif key == ord("l"):
                    self.edge = LOWER
                    self.print_status()
                elif key == ord("a"):
                    self.adding = True
                elif key == ord("r"):
                    auto_key = "auto_" + self.edge
                    self.current()[self.edge] = json.loads(json.dumps(self.current()[auto_key]))
                    self.current()["reviewed"] = False
                elif key == ord("c"):
                    self.current()[self.edge] = []
                    self.current()["reviewed"] = False
                elif key in (ord("p"), ord("[")):
                    self.save()
                    self.index = max(0, self.index - 1)
                    self.print_status()
                elif key in (ord(" "), 13, ord("n"), ord("]")):
                    usable_upper = len(self.current()[UPPER]) >= 2
                    usable_lower = len(self.current()[LOWER]) >= 2
                    if not usable_upper and not usable_lower:
                        print("pose {:02d}: retain at least one edge with two points".format(
                            self.current()["id"]))
                        continue
                    self.current()["reviewed"] = True
                    self.save()
                    if self.index + 1 < len(self.entries):
                        self.index += 1
                        self.print_status()
                    else:
                        print("All poses visited; press Q to finish or P to review previous poses.")
        finally:
            cv2.destroyAllWindows()


def render_auto_overlay(image: np.ndarray, lines: dict, pose_id: int) -> np.ndarray:
    view = image.copy()
    for edge in (UPPER, LOWER):
        points = np.rint(lines[edge]).astype(np.int32)
        if len(points) >= 2:
            cv2.polylines(view, [points], False, COLOURS[edge], 3, cv2.LINE_AA)
        for point in points:
            cv2.circle(view, tuple(point), 6, COLOURS[edge], -1)
    cv2.putText(view, "pose {:02d}: automatic RANSAC initialization".format(pose_id),
                (20, 32), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def prepare(args) -> tuple[dict, Path]:
    session_path = args.session / "poses.json"
    session = json.loads(session_path.read_text(encoding="utf-8"))
    tags_path = args.tags or args.session / "arena_fence_lines.json"
    if tags_path.exists() and not args.reset:
        existing_tags = json.loads(tags_path.read_text(encoding="utf-8"))
        if existing_tags.get("format_version") != 2:
            raise ValueError("{} uses an old format; rerun with --reset".format(tags_path))
        print("loaded existing tags without rerunning rectification or RANSAC: {}".format(tags_path))
        return existing_tags, tags_path

    poses = [pose for pose in session["poses"] if pose.get("accepted")]
    width = int(session["capture"]["actual_width"])
    height = int(session["capture"]["actual_height"])
    calibration_path = args.session / session.get("camera_calibration", {}).get(
        "path", "camera-calibration.yaml")
    maps, rectified_matrix = load_calibration(calibration_path, width, height)
    ranges = load_blue_ranges(args.hsv)
    rectified_dir = args.session / "rectified"
    automatic_dir = args.session / "automatic-lines"
    rectified_dir.mkdir(exist_ok=True)
    automatic_dir.mkdir(exist_ok=True)

    entries = []
    for pose in poses:
        rectified_path = rectified_dir / "pose-{:02d}.jpg".format(pose["id"])
        image = rectified_median(args.session, pose, maps)
        cv2.imwrite(str(rectified_path), image, [cv2.IMWRITE_JPEG_QUALITY, 96])
        mask = largest_blue_mask(image, ranges)
        lines = make_auto_points(mask, pose["id"])
        entry = {
            "id": pose["id"], "reviewed": False,
            "x_m": pose["x_m"], "y_m": pose["y_m"], "yaw_deg": pose["yaw_deg"],
            "rectified_image": str(rectified_path.relative_to(args.session)),
            UPPER: lines[UPPER], LOWER: lines[LOWER],
        }
        entry["auto_upper"] = lines[UPPER]
        entry["auto_lower"] = lines[LOWER]
        entries.append(entry)
        overlay = render_auto_overlay(image, lines, pose["id"])
        cv2.imwrite(str(automatic_dir / "pose-{:02d}.jpg".format(pose["id"])), overlay,
                    [cv2.IMWRITE_JPEG_QUALITY, 94])
        print("pose {:02d}: auto upper={} lower={}".format(
            pose["id"], len(lines[UPPER]), len(lines[LOWER])))
    tags = {
        "format_version": 2,
        "source": str(session_path),
        "projection": "rectilinear",
        "rectified_K": rectified_matrix.tolist(),
        "poses": entries,
    }
    atomic_write_json(tags_path, tags)
    return tags, tags_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--hsv", type=Path, required=True)
    parser.add_argument("--tags", type=Path)
    parser.add_argument("--reset", action="store_true",
                        help="discard prior edits and initialize all lines again")
    parser.add_argument("--prepare-only", action="store_true",
                        help="write rectified images and automatic overlays without opening the GUI")
    args = parser.parse_args()
    try:
        tags, tags_path = prepare(args)
        print("wrote {}".format(tags_path))
        if not args.prepare_only:
            Reviewer(tags, tags_path, args.session).run()
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError, cv2.error) as error:
        print("tagging failed: {}".format(error))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
