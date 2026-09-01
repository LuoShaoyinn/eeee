#!/usr/bin/env python3
"""Build a normalized YOLO train/validation dataset from reviewed labels."""

import argparse
import random
import shutil
from pathlib import Path

from review_labels import read_manifest


NAMES = ("other_robot", "red_cube", "yellow_cylinder", "home")


def load_boxes(path: Path) -> list[tuple[int, float, float, float, float]]:
    boxes = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{number}: expected five fields")
        class_id, cx, cy, width, height = map(float, fields)
        if class_id != int(class_id) or not 0 <= class_id < len(NAMES) or width <= 0 or height <= 0:
            raise ValueError(f"{path}:{number}: invalid YOLO box")
        left, top = max(0.0, cx - width / 2), max(0.0, cy - height / 2)
        right, bottom = min(1.0, cx + width / 2), min(1.0, cy + height / 2)
        if right <= left or bottom <= top:
            raise ValueError(f"{path}:{number}: box outside image")
        boxes.append((int(class_id), (left + right) / 2, (top + bottom) / 2,
                      right - left, bottom - top))
    return boxes


def write_split(records: list[tuple[Path, list[tuple[int, float, float, float, float]]]], output: Path) -> None:
    images, labels = output / "images", output / "labels"
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)
    for image, boxes in records:
        target = images / image.name
        target.unlink(missing_ok=True)
        target.symlink_to(image.resolve())
        (labels / f"{image.stem}.txt").write_text("".join(
            f"{class_id} {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}\n"
            for class_id, cx, cy, width, height in boxes))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("dataset/reviewed_images.txt"))
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--labels", type=Path, default=Path("dataset/detection_labels"))
    parser.add_argument("--output", type=Path, default=Path("dataset/yolo"))
    parser.add_argument("--validation-fraction", type=float, default=.2)
    parser.add_argument("--seed", type=int, default=314)
    args = parser.parse_args()
    stems = sorted(read_manifest(args.manifest, args.images))
    records = []
    for stem in stems:
        image = next((args.images / f"{stem}{suffix}" for suffix in (".jpg", ".jpeg", ".png")
                      if (args.images / f"{stem}{suffix}").exists()), None)
        label = args.labels / f"{stem}.txt"
        if image is None or not label.exists():
            raise ValueError(f"missing reviewed image or label for {stem}")
        records.append((image, load_boxes(label)))

    random.Random(args.seed).shuffle(records)
    validation_count = round(len(records) * args.validation_fraction)
    validation, train = records[:validation_count], records[validation_count:]
    if args.output.exists():
        shutil.rmtree(args.output)
    write_split(train, args.output / "train")
    write_split(validation, args.output / "val")
    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\ntrain: train/images\nval: val/images\nnames: {list(NAMES)!r}\n")
    print(f"wrote {len(train)} train and {len(validation)} validation images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
