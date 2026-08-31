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
    parser.add_argument("--width", type=int,
                        help="capture width (640 raw default, 1280 rectified default)")
    parser.add_argument("--height", type=int,
                        help="capture height (480 raw default, 720 rectified default)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration", type=float, default=30,
                        help="sample length in seconds (default: 30)")
    parser.add_argument("--output", type=Path,
                        help="destination .mjpg file (default: ~/videos/capture-<time>.mjpg)")
    parser.add_argument("--rectify", action="store_true",
                        help="apply the Camera1 1280x720 fisheye calibration before saving")
    parser.add_argument("--calibration", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration" /
                                "camera1_fisheye_1280x720_rectilinear_f400.yaml")
    args = parser.parse_args()

    width = args.width or (1280 if args.rectify else 640)
    height = args.height or (720 if args.rectify else 480)
    if min(width, height, args.fps, args.duration) <= 0:
        parser.error("width, height, fps, and duration must be positive")
    if shutil.which("v4l2-ctl") is None:
        parser.error("v4l2-ctl is required; install the v4l-utils package")

    extension = ".avi" if args.rectify else ".mjpg"
    output = args.output or (Path.home() / "videos" /
                             f"capture-{datetime.now():%Y%m%d-%H%M%S}{extension}")
    output.parent.mkdir(parents=True, exist_ok=True)
    frame_count = round(args.duration * args.fps)

    if args.rectify:
        if not args.calibration.is_file():
            parser.error(f"calibration file not found: {args.calibration}")
        source = Path(__file__).with_name("cubie_video_capture.cpp")
        recorder = Path(__file__).with_name("cubie_video_capture")
        if not recorder.exists() or recorder.stat().st_mtime < source.stat().st_mtime:
            compile_command = ["g++", "-O3", "-std=c++17", str(source), "-o", str(recorder)]
            try:
                opencv_flags = subprocess.check_output(
                    ["pkg-config", "--cflags", "--libs", "opencv4"], text=True).split()
                subprocess.run(compile_command + opencv_flags, check=True)
            except (OSError, subprocess.CalledProcessError) as error:
                print(f"cannot build calibrated recorder: {error}", file=sys.stderr)
                return 1
        capture = [str(recorder), "--device", args.device, "--output", str(output),
                   "--calibration", str(args.calibration), "--width", str(width),
                   "--height", str(height), "--fps", str(args.fps), "--frames", str(frame_count)]
        try:
            subprocess.run(capture, check=True)
        except subprocess.CalledProcessError as error:
            print(f"capture failed: {error}", file=sys.stderr)
            return 1
        print(f"saved rectified {output} ({output.stat().st_size} bytes)")
        return 0

    configure = [
        "v4l2-ctl", "--device", args.device,
        f"--set-fmt-video=width={width},height={height},pixelformat=MJPG",
        f"--set-parm={args.fps}",
    ]
    capture = [
        "v4l2-ctl", "--device", args.device,
        "--stream-mmap=3", f"--stream-count={frame_count}",
        f"--stream-to={output}",
    ]

    try:
        subprocess.run(configure, check=True)
        print(f"capturing {frame_count} frames at {width}x{height} {args.fps} FPS")
        subprocess.run(capture, check=True)
    except subprocess.CalledProcessError as error:
        print(f"capture failed: {error}", file=sys.stderr)
        return 1

    print(f"saved {output} ({output.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
