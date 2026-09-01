#!/usr/bin/env python3
"""Extract evenly sampled JPEG frames from one or more video files."""

import argparse
from pathlib import Path

import cv2


def extract(video_path: Path, output_dir: Path, sample_fps: float, quality: int) -> int:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video_path}")

    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if source_fps <= 0:
        raise RuntimeError(f"cannot determine FPS for {video_path}")
    frame_interval = max(1, round(source_fps / sample_fps))

    frame_index = 0
    saved = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % frame_interval == 0:
            name = f"{video_path.stem}_{saved:05d}.jpg"
            destination = output_dir / name
            if not cv2.imwrite(str(destination), frame,
                               [cv2.IMWRITE_JPEG_QUALITY, quality]):
                raise RuntimeError(f"cannot write {destination}")
            saved += 1
        frame_index += 1

    capture.release()
    print(f"{video_path.name}: saved {saved} frames at {sample_fps:g} FPS")
    return saved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("dataset/images"))
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--quality", type=int, default=95)
    args = parser.parse_args()
    if args.fps <= 0 or not 1 <= args.quality <= 100:
        parser.error("--fps must be positive and --quality must be 1..100")

    args.output.mkdir(parents=True, exist_ok=True)
    total = sum(extract(path, args.output, args.fps, args.quality) for path in args.videos)
    print(f"total: {total} frames in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
