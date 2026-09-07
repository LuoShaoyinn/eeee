#!/usr/bin/env python3
"""Windows-friendly checkpoint-assisted object annotation entry point."""

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("dataset"))
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--confidence", type=float, default=0.15)
    parser.add_argument("--device", default=None)
    parser.add_argument("--refresh", action="store_true", help="regenerate proposals before reviewing")
    parser.add_argument("--include", default="*.jpg")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    images = args.dataset / "images"
    labels = args.dataset / "detection_labels"
    candidates = args.dataset / "detection_candidates"
    manifest = args.dataset / "reviewed_images.txt"
    if not images.is_dir():
        parser.error(f"image directory does not exist: {images}")
    if not args.model.is_file():
        parser.error(f"checkpoint does not exist: {args.model}")

    proposal = root / "tools" / "dataset" / "propose_yolo_boxes.py"
    reviewer = root / "tools" / "dataset" / "review_labels.py"
    if args.refresh or not candidates.exists() or not any(candidates.glob("*.txt")):
        command = [sys.executable, str(proposal), "--model", str(args.model), "--images", str(images),
                   "--output", str(candidates), "--confidence", str(args.confidence),
                   "--include", args.include]
        if args.device is not None:
            command.extend(("--device", args.device))
        subprocess.run(command, check=True)
    return subprocess.run([sys.executable, str(reviewer), "--detection-only", "--images", str(images),
                           "--labels", str(labels), "--proposals", str(candidates),
                           "--reviewed-manifest", str(manifest), "--include", args.include], check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
