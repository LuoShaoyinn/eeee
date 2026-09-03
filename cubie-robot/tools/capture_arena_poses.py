#!/usr/bin/env python3
"""Capture raw camera bursts at manually measured robot poses in the arena."""

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_pose(text: str) -> tuple[float, float, float, str]:
    fields = text.split(maxsplit=3)
    if len(fields) < 3:
        raise ValueError("enter: x_m y_m yaw_deg [note]")
    return float(fields[0]), float(fields[1]), float(fields[2]), fields[3] if len(fields) == 4 else ""


def print_poses(session: dict) -> None:
    poses = session["poses"]
    if not poses:
        print("No poses captured yet.")
        return
    for pose in poses:
        state = "accepted" if pose["accepted"] else "rejected"
        print("  {:02d}: x={:.3f} y={:.3f} yaw={:.1f} deg, {} frames, {}{}".format(
            pose["id"], pose["x_m"], pose["y_m"], pose["yaw_deg"],
            len(pose["images"]), state,
            " - " + pose["note"] if pose["note"] else ""))


def capture_burst(capture, args, pose_id: int, output_dir: Path) -> list[dict]:
    for _ in range(args.flush_frames):
        ok, _ = capture.read()
        if not ok:
            raise RuntimeError("camera read failed while flushing buffered frames")

    pose_dir = output_dir / "images" / "pose-{:02d}".format(pose_id)
    pose_dir.mkdir(parents=True, exist_ok=False)
    images = []
    for frame_index in range(args.images_per_pose):
        ok, frame = capture.read()
        captured_ns = time.monotonic_ns()
        if not ok or frame is None:
            raise RuntimeError("camera read failed")
        if frame.shape[1] != args.width or frame.shape[0] != args.height:
            raise RuntimeError("camera returned {}x{}, expected {}x{}".format(
                frame.shape[1], frame.shape[0], args.width, args.height))

        filename = "images/pose-{0:02d}/raw-{1:02d}.jpg".format(pose_id, frame_index)
        path = output_dir / filename
        parameters = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
        if not cv2.imwrite(str(path), frame, parameters):
            raise RuntimeError("cannot write {}".format(path))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        images.append({
            "path": filename,
            "monotonic_ns": captured_ns,
            "sharpness_laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        })
        print("  saved {} ({}/{})".format(filename, frame_index + 1, args.images_per_pose),
              flush=True)
        if frame_index + 1 < args.images_per_pose:
            time.sleep(args.interval)
    return images


def new_session(args, actual_width: int, actual_height: int, actual_fps: float) -> dict:
    return {
        "format_version": 1,
        "created_at_utc": utc_now(),
        "coordinate_system": {
            "origin": "arena corner A=(0,0)",
            "x": "toward B along the 3.0 m side",
            "y": "toward C along the 1.985 m side",
            "yaw_deg": "0 faces +x; positive rotates counter-clockwise toward +y",
            "pose_reference": "chassis centre projected onto the floor",
        },
        "arena": {"x_length_m": args.arena_x, "y_length_m": args.arena_y},
        "capture": {
            "device": args.device,
            "requested_width": args.width,
            "requested_height": args.height,
            "requested_fps": args.fps,
            "actual_width": actual_width,
            "actual_height": actual_height,
            "actual_fps": actual_fps,
            "image_encoding": "JPEG",
            "jpeg_quality": args.jpeg_quality,
            "images_per_pose": args.images_per_pose,
            "raw_fisheye": True,
        },
        "poses": [],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--output-dir", type=Path,
                        default=Path.home() / "arena-calibration" /
                                datetime.now().strftime("session-%Y%m%d-%H%M%S"))
    parser.add_argument("--calibration", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration" /
                                "camera2_fisheye_1280x720.yaml",
                        help="YAML to archive with this raw-image session")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--images-per-pose", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.15)
    parser.add_argument("--flush-frames", type=int, default=12)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--target-poses", type=int, default=10)
    parser.add_argument("--arena-x", type=float, default=3.0)
    parser.add_argument("--arena-y", type=float, default=1.985)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps, args.images_per_pose, args.flush_frames,
           args.target_poses, args.arena_x, args.arena_y) <= 0:
        parser.error("dimensions, counts, frame rate, and arena lengths must be positive")
    if args.interval < 0 or not 1 <= args.jpeg_quality <= 100:
        parser.error("interval must be non-negative and JPEG quality must be in [1, 100]")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = args.output_dir / "poses.json"

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if not capture.isOpened():
        print("capture failed: cannot open {}".format(args.device), file=sys.stderr)
        return 1

    try:
        actual_width = round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        if (actual_width, actual_height) != (args.width, args.height):
            raise RuntimeError("camera negotiated {}x{}, expected {}x{}".format(
                actual_width, actual_height, args.width, args.height))

        if metadata_path.exists():
            session = json.loads(metadata_path.read_text(encoding="utf-8"))
            if session.get("format_version") != 1 or not isinstance(session.get("poses"), list):
                raise RuntimeError("unsupported session metadata in {}".format(metadata_path))
            print("Resuming {} with {} recorded pose(s).".format(
                args.output_dir, len(session["poses"])))
        else:
            session = new_session(args, actual_width, actual_height, actual_fps)
            if args.calibration.is_file():
                calibration_copy = args.output_dir / "camera-calibration.yaml"
                shutil.copy2(args.calibration, calibration_copy)
                session["camera_calibration"] = {
                    "path": calibration_copy.name,
                    "source": str(args.calibration),
                    "sha256": file_sha256(calibration_copy),
                }
            else:
                print("Warning: calibration YAML not found; raw frames can still be captured: {}".format(
                    args.calibration), file=sys.stderr)
            atomic_write_json(metadata_path, session)

        print("Camera ready: {}x{} at reported {:.1f} FPS.".format(
            actual_width, actual_height, actual_fps))
        print("Measure the chassis centre. Enter: x_m y_m yaw_deg [note]")
        print("Commands: list, undo (reject last pose), q")
        while True:
            accepted_count = sum(bool(pose["accepted"]) for pose in session["poses"])
            prompt = "pose {}/{} > ".format(accepted_count + 1, args.target_poses)
            try:
                command = input(prompt).strip()
            except EOFError:
                command = "q"
            if not command:
                continue
            if command.lower() in {"q", "quit", "done"}:
                break
            if command.lower() == "list":
                print_poses(session)
                continue
            if command.lower() == "undo":
                accepted = [pose for pose in session["poses"] if pose["accepted"]]
                if not accepted:
                    print("No accepted pose to reject.")
                    continue
                accepted[-1]["accepted"] = False
                accepted[-1]["rejected_at_utc"] = utc_now()
                atomic_write_json(metadata_path, session)
                print("Pose {:02d} marked rejected; its images were retained.".format(accepted[-1]["id"]))
                continue
            try:
                x_m, y_m, yaw_deg, note = parse_pose(command)
            except ValueError as error:
                print("Invalid pose: {}".format(error))
                continue
            if not 0.0 <= x_m <= args.arena_x or not 0.0 <= y_m <= args.arena_y:
                print("Pose is outside the {:.3f} x {:.3f} m arena.".format(
                    args.arena_x, args.arena_y))
                continue
            print("x={:.3f} m, y={:.3f} m, yaw={:.1f} deg{}".format(
                x_m, y_m, yaw_deg, " - " + note if note else ""))
            confirmation = input("Press Enter to capture; type r to re-enter or q to finish: ").strip().lower()
            if confirmation in {"q", "quit"}:
                break
            if confirmation:
                continue

            pose_id = max((pose["id"] for pose in session["poses"]), default=0) + 1
            images = capture_burst(capture, args, pose_id, args.output_dir)
            session["poses"].append({
                "id": pose_id,
                "accepted": True,
                "captured_at_utc": utc_now(),
                "x_m": x_m,
                "y_m": y_m,
                "yaw_deg": yaw_deg,
                "note": note,
                "images": images,
            })
            atomic_write_json(metadata_path, session)
            accepted_count += 1
            print("Pose {:02d} committed to {} ({} accepted total).".format(
                pose_id, metadata_path, accepted_count))
            if accepted_count >= args.target_poses:
                print("Target reached. Add another pose, or enter q to finish.")
    except (OSError, RuntimeError, json.JSONDecodeError, cv2.error) as error:
        print("capture failed: {}".format(error), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nCapture stopped; all completed poses are already saved.")
    finally:
        capture.release()

    accepted_count = sum(bool(pose["accepted"]) for pose in session["poses"])
    print("Session complete: {} accepted pose(s) in {}".format(accepted_count, args.output_dir))
    return 0 if accepted_count else 1


if __name__ == "__main__":
    raise SystemExit(main())
