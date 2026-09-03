#!/usr/bin/env python3
"""Run sampled video frames through the A733 YOLOv5 NPU demo safely."""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


def npu_temperature_c() -> float:
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            if (zone / "type").read_text().strip() == "npu_thermal_zone":
                return int((zone / "temp").read_text().strip()) / 1000.0
        except OSError:
            continue
    raise RuntimeError("could not read npu_thermal_zone")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--deploy-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--sample-fps", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=30)
    parser.add_argument("--max-temp-c", type=float, default=70.0)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.sample_fps <= 0 or args.max_frames <= 0:
        raise ValueError("sample-fps and max-frames must be positive")
    if not args.video.is_file():
        raise FileNotFoundError(args.video)

    deploy_dir = args.deploy_dir.resolve()
    demo = deploy_dir / "yolov5_demo_a733"
    model = deploy_dir / "yolov5s_rt_uint8_a733.nb"
    for required in (demo, model, deploy_dir / "libNBGlinker.so", deploy_dir / "libVIPhal.so"):
        if not required.is_file():
            raise FileNotFoundError(f"missing deployment file: {required}")

    starting_temp = npu_temperature_c()
    if starting_temp >= args.max_temp_c:
        raise RuntimeError(f"NPU is already {starting_temp:.1f}C; limit is {args.max_temp_c:.1f}C")

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    output_dir = args.output_dir or deploy_dir / f"video-verify-{stamp}"
    frames_dir = output_dir / "frames"
    logs_dir = output_dir / "logs"
    frames_dir.mkdir(parents=True)
    logs_dir.mkdir()

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"could not open video: {args.video}")
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if source_fps <= 0:
        source_fps = args.sample_fps
    stride = max(1, round(source_fps / args.sample_fps))

    temperatures = output_dir / "temperatures.csv"
    with temperatures.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("sample", "source_frame", "before_c", "after_c", "elapsed_s", "returncode"))
        source_frame = 0
        sample = 0
        while sample < args.max_frames:
            ok, image = capture.read()
            if not ok:
                break
            if source_frame % stride:
                source_frame += 1
                continue

            before_c = npu_temperature_c()
            if before_c >= args.max_temp_c:
                print(f"stopping at {before_c:.1f}C before sample {sample}", file=sys.stderr)
                return 2

            frame_path = frames_dir / f"frame-{sample:05d}.jpg"
            if not cv2.imwrite(str(frame_path), image):
                raise RuntimeError(f"could not write {frame_path}")
            started = time.monotonic()
            result = subprocess.run(
                [str(demo), "-nb", str(model), "-i", str(frame_path), "-l", "1"],
                cwd=deploy_dir,
                env={"LD_LIBRARY_PATH": str(deploy_dir)},
                text=True,
                capture_output=True,
            )
            elapsed_s = time.monotonic() - started
            (logs_dir / f"frame-{sample:05d}.log").write_text(result.stdout + result.stderr)
            after_c = npu_temperature_c()
            writer.writerow((sample, source_frame, f"{before_c:.1f}", f"{after_c:.1f}", f"{elapsed_s:.3f}", result.returncode))
            csv_file.flush()
            print(f"sample {sample}: {before_c:.1f}C -> {after_c:.1f}C, {elapsed_s:.3f}s")
            if result.returncode:
                return result.returncode
            if after_c >= args.max_temp_c:
                print(f"stopping at {after_c:.1f}C after sample {sample}", file=sys.stderr)
                return 2
            sample += 1
            source_frame += 1

    capture.release()
    print(f"completed {sample} samples; results: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
