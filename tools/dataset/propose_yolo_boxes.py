#!/usr/bin/env python3
"""Generate review-only YOLO box proposals from an Ultralytics checkpoint."""

import argparse
from pathlib import Path

import cv2


def iou(first: tuple[float, ...], second: tuple[float, ...]) -> float:
    """Return IoU for ``(class, confidence, left, top, right, bottom)`` boxes."""
    _, _, left_a, top_a, right_a, bottom_a = first
    _, _, left_b, top_b, right_b, bottom_b = second
    intersection = max(0.0, min(right_a, right_b) - max(left_a, left_b))
    intersection *= max(0.0, min(bottom_a, bottom_b) - max(top_a, top_b))
    if not intersection:
        return 0.0
    area_a = (right_a - left_a) * (bottom_a - top_a)
    area_b = (right_b - left_b) * (bottom_b - top_b)
    return intersection / (area_a + area_b - intersection)


def merge_same_class(detections: list[tuple[float, ...]], threshold: float) -> list[tuple[float, ...]]:
    """Merge connected groups of high-IoU detections from the same class.

    Ultralytics already runs NMS. This removes the remaining duplicated proposals
    without ever merging boxes from different classes.
    """
    merged = []
    for class_id in sorted({int(item[0]) for item in detections}):
        group = [item for item in detections if int(item[0]) == class_id]
        remaining = set(range(len(group)))
        while remaining:
            component = {remaining.pop()}
            pending = list(component)
            while pending:
                current = pending.pop()
                adjacent = {index for index in remaining if iou(group[current], group[index]) >= threshold}
                remaining -= adjacent
                component |= adjacent
                pending.extend(adjacent)
            members = [group[index] for index in component]
            confidence_sum = sum(item[1] for item in members)
            # Confidence weighting keeps an outlying weak proposal from moving a strong box.
            weights = [item[1] / confidence_sum if confidence_sum else 1 / len(members) for item in members]
            merged.append((class_id, max(item[1] for item in members),
                           *(sum(weight * item[column] for weight, item in zip(weights, members))
                             for column in range(2, 6))))
    return merged


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--images", type=Path, default=Path("dataset/images"))
    parser.add_argument("--include", default="*.jpg")
    parser.add_argument("--output", type=Path, default=Path("dataset/detection_candidates"))
    parser.add_argument("--preview", type=Path, default=Path("dataset/candidate_previews"))
    parser.add_argument("--confidence", type=float, default=0.15)
    parser.add_argument("--merge-iou", type=float, default=0.70)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None, help="Ultralytics device, e.g. cpu or 0")
    args = parser.parse_args()
    if not 0 < args.confidence < 1 or not 0 < args.merge_iou < 1:
        parser.error("confidence and merge IoU must be between zero and one")

    from ultralytics import YOLO

    images = sorted(path for path in args.images.glob(args.include)
                    if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        parser.error(f"no images match {args.include!r} in {args.images}")
    args.output.mkdir(parents=True, exist_ok=True)
    args.preview.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.model)
    for result in model.predict(images, stream=True, conf=args.confidence, imgsz=args.imgsz,
                                device=args.device, verbose=False):
        path = Path(result.path)
        image = cv2.imread(str(path))
        height, width = image.shape[:2]
        detections = []
        for xyxy, confidence, class_id in zip(result.boxes.xyxy.cpu().tolist(),
                                               result.boxes.conf.cpu().tolist(),
                                               result.boxes.cls.cpu().tolist()):
            left, top, right, bottom = xyxy
            detections.append((int(class_id), confidence, max(0.0, left), max(0.0, top),
                               min(float(width), right), min(float(height), bottom)))
        lines = []
        for class_id, _confidence, left, top, right, bottom in merge_same_class(detections, args.merge_iou):
            if right <= left or bottom <= top:
                continue
            lines.append(f"{class_id} {(left + right) / (2 * width):.6f} "
                         f"{(top + bottom) / (2 * height):.6f} {(right - left) / width:.6f} "
                         f"{(bottom - top) / height:.6f}")
        (args.output / f"{path.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
    print(f"wrote proposals for {len(images)} images to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
