#!/usr/bin/env python3
"""Capture a fixed-duration MJPEG sample locally through V4L2."""

import argparse
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Headless V4L2 capture to a local MJPEG file without decoding.")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration", type=float, default=30,
                        help="sample length in seconds (default: 30)")
    parser.add_argument("--output", type=Path,
                        help="destination .mjpg file (default: ~/videos/capture-<time>.mjpg)")
    args = parser.parse_args()

    if min(args.width, args.height, args.fps, args.duration) <= 0:
        parser.error("width, height, fps, and duration must be positive")
    if shutil.which("v4l2-ctl") is None:
        parser.error("v4l2-ctl is required; install the v4l-utils package")

    output = args.output or (Path.home() / "videos" /
                             f"capture-{datetime.now():%Y%m%d-%H%M%S}.mjpg")
    output.parent.mkdir(parents=True, exist_ok=True)
    frame_count = round(args.duration * args.fps)

    configure = [
        "v4l2-ctl", "--device", args.device,
        f"--set-fmt-video=width={args.width},height={args.height},pixelformat=MJPG",
        f"--set-parm={args.fps}",
    ]
    capture = [
        "v4l2-ctl", "--device", args.device,
        "--stream-mmap=3", f"--stream-count={frame_count}",
        f"--stream-to={output}",
    ]

    try:
        subprocess.run(configure, check=True)
        print(f"capturing {frame_count} frames at {args.width}x{args.height} {args.fps} FPS")
        subprocess.run(capture, check=True)
    except subprocess.CalledProcessError as error:
        print(f"capture failed: {error}", file=sys.stderr)
        return 1

    print(f"saved {output} ({output.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
