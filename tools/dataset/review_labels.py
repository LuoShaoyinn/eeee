#!/usr/bin/env python3
"""OpenCV reviewer for semantic masks and YOLO object boxes."""

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

SEMANTIC = {"other": 0, "blue_fence": 1, "white_ground": 2, "home_black": 3}
DETECTION = {"other_robot": 0, "red_cube": 1, "yellow_cylinder": 2, "home": 3}
SEMANTIC_COLORS = {0: (0, 0, 0), 1: (255, 0, 0), 2: (255, 255, 255), 3: (255, 0, 255)}
BOX_COLORS = {0: (0, 255, 0), 1: (0, 0, 255), 2: (0, 255, 255), 3: (255, 0, 255)}


@dataclass
class Box:
    class_id: int
    x: int
    y: int
    w: int
    h: int


def read_manifest(path: Path | None, images: Path) -> set[str]:
    if path is None or not path.exists():
        return set()
    stems = set()
    for line in path.read_text().splitlines():
        entry = line.strip()
        if not entry or entry.startswith("#"):
            continue
        if any(character in entry for character in "*?["):
            stems.update(candidate.stem for candidate in images.glob(entry))
        else:
            stems.add(Path(entry).stem)
    return stems


class Reviewer:
    def __init__(self, images: list[Path], masks: Path, labels: Path, brush: int,
                 detection_only: bool, proposals: Path | None, reviewed_manifest: Path | None):
        self.images, self.masks, self.labels, self.brush = images, masks, labels, brush
        self.detection_only = detection_only
        self.proposals = proposals
        self.reviewed_manifest = reviewed_manifest
        self.reviewed_stems = read_manifest(reviewed_manifest, images[0].parent)
        self.index, self.mode, self.class_id = 0, "box" if detection_only else "mask", 1
        self.image: np.ndarray | None = None
        self.mask: np.ndarray | None = None
        self.boxes: list[Box] = []
        self.drag_start: tuple[int, int] | None = None
        self.loaded_proposal = False
        self.load()

    @property
    def stem(self) -> str:
        return self.images[self.index].stem

    def load(self) -> None:
        self.image = cv2.imread(str(self.images[self.index]))
        height, width = self.image.shape[:2]
        mask_path = self.masks / f"{self.stem}.png"
        self.mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if self.mask is None or self.mask.shape != (height, width):
            self.mask = np.zeros((height, width), dtype=np.uint8)
        self.boxes = []
        label_path = self.labels / f"{self.stem}.txt"
        self.loaded_proposal = False
        if self.stem not in self.reviewed_stems and self.proposals is not None:
            candidate = self.proposals / f"{self.stem}.txt"
            if candidate.exists():
                label_path = candidate
                self.loaded_proposal = True
        if label_path.exists():
            for line in label_path.read_text().splitlines():
                values = line.split()
                if len(values) != 5:
                    continue
                class_id, cx, cy, bw, bh = map(float, values)
                w, h = round(bw * width), round(bh * height)
                self.boxes.append(Box(int(class_id), round(cx * width - w / 2),
                                      round(cy * height - h / 2), w, h))

    def save(self) -> None:
        if not self.detection_only:
            self.masks.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(self.masks / f"{self.stem}.png"), self.mask)
        self.labels.mkdir(parents=True, exist_ok=True)
        height, width = self.image.shape[:2]
        lines = []
        for box in self.boxes:
            cx, cy = (box.x + box.w / 2) / width, (box.y + box.h / 2) / height
            lines.append(f"{box.class_id} {cx:.6f} {cy:.6f} {box.w / width:.6f} {box.h / height:.6f}")
        (self.labels / f"{self.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        if self.reviewed_manifest is not None and self.stem not in self.reviewed_stems:
            with self.reviewed_manifest.open("a") as manifest:
                manifest.write(f"{self.stem}\n")
            self.reviewed_stems.add(self.stem)

    def display(self) -> np.ndarray:
        canvas = self.image.copy()
        if not self.detection_only:
            colors = np.zeros_like(canvas)
            for class_id, color in SEMANTIC_COLORS.items():
                colors[self.mask == class_id] = color
            canvas = cv2.addWeighted(canvas, .65, colors, .35, 0)
        for box in self.boxes:
            cv2.rectangle(canvas, (box.x, box.y), (box.x + box.w, box.y + box.h),
                          BOX_COLORS[box.class_id], 2)
            cv2.putText(canvas, list(DETECTION)[box.class_id], (box.x, max(16, box.y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, BOX_COLORS[box.class_id], 2)
        mode = (f"{self.mode}: {list(SEMANTIC)[self.class_id]}" if self.mode == "mask"
                else f"{self.mode}: {list(DETECTION)[self.class_id]}")
        source = "proposal" if self.loaded_proposal else "verified"
        title = f"{self.index + 1}/{len(self.images)} {self.stem} | {mode} | {source}"
        cv2.putText(canvas, title, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 3)
        cv2.putText(canvas, title, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 1)
        return canvas

    def mouse(self, event: int, x: int, y: int, flags: int, _param: object) -> None:
        if self.mode == "mask":
            if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
                cv2.circle(self.mask, (x, y), self.brush, self.class_id, -1)
            elif event == cv2.EVENT_RBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_RBUTTON):
                cv2.circle(self.mask, (x, y), self.brush, SEMANTIC["other"], -1)
        elif self.mode == "box":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag_start = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and self.drag_start:
                start_x, start_y = self.drag_start
                left, top = min(start_x, x), min(start_y, y)
                width, height = abs(x - start_x), abs(y - start_y)
                if width >= 8 and height >= 8:
                    self.boxes.append(Box(self.class_id, left, top, width, height))
                self.drag_start = None
        elif self.mode == "delete" and event == cv2.EVENT_LBUTTONDOWN:
            self.boxes = [box for box in self.boxes
                          if not (box.x <= x <= box.x + box.w and box.y <= y <= box.y + box.h)]

    def next(self, delta: int) -> None:
        self.save()
        self.index = max(0, min(len(self.images) - 1, self.index + delta))
        self.load()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--masks", type=Path, default=Path("dataset/semantic_masks"))
    parser.add_argument("--labels", type=Path, default=Path("dataset/detection_labels"))
    parser.add_argument("--brush", type=int, default=18)
    parser.add_argument("--detection-only", action="store_true",
                        help="edit YOLO boxes only; never write semantic masks")
    parser.add_argument("--proposals", type=Path,
                        help="unverified YOLO boxes displayed for frames outside the reviewed manifest")
    parser.add_argument("--reviewed-manifest", type=Path,
                        help="verified frame manifest; saving appends the current frame")
    parser.add_argument("--include", default="*",
                        help="filename glob used to limit the image set, e.g. capture-20260901-*.jpg")
    args = parser.parse_args()
    images = sorted(path for path in args.images.glob(args.include)
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        parser.error(f"no images found in {args.images}")

    reviewer = Reviewer(images, args.masks, args.labels, args.brush, args.detection_only,
                        args.proposals, args.reviewed_manifest)
    window = "Dataset reviewer"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, reviewer.mouse)
    print("keys: 0/1/2/3 semantic class, r/y/o/h box class, m mask mode, b box mode, "
          "x delete boxes, [/] previous/next, s save, q quit")
    while True:
        cv2.imshow(window, reviewer.display())
        key = cv2.waitKey(16) & 0xFF
        if key == 255:
            continue
        if key in (ord("q"), 27):
            reviewer.save()
            break
        if key == ord("s"):
            reviewer.save()
        elif key == ord("["):
            reviewer.next(-1)
        elif key == ord("]"):
            reviewer.next(1)
        elif not args.detection_only and key in (ord("0"), ord("1"), ord("2"), ord("3")):
            reviewer.mode, reviewer.class_id = "mask", int(chr(key))
        elif key in (ord("r"), ord("y"), ord("o"), ord("h")):
            reviewer.mode = "box"
            reviewer.class_id = {ord("o"): 0, ord("r"): 1, ord("y"): 2, ord("h"): 3}[key]
        elif not args.detection_only and key == ord("m"):
            reviewer.mode = "mask"
        elif key == ord("b"):
            reviewer.mode = "box"
        elif key == ord("x"):
            reviewer.mode = "delete"
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
