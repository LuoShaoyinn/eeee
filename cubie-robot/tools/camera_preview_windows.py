#!/usr/bin/env python3
"""Host-side live preview with Space-to-capture for Cubie camera calibration."""

import argparse
import getpass
import shlex
import struct
import sys
from pathlib import Path

import cv2
import numpy as np


REMOTE_STREAM = r"""
import cv2
import os
import select
import struct
import sys
import numpy as np

device, calibration, output_dir, width, height, fps, raw_mode = sys.argv[1:]
width, height, fps = int(width), int(height), int(fps)
raw_mode = raw_mode == "1"
if not raw_mode:
    storage = cv2.FileStorage(calibration, cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise SystemExit("cannot open calibration: " + calibration)
    K = storage.getNode("K").mat()
    D = storage.getNode("D").mat()
    rectified_K = storage.getNode("rectified_K").mat()
    calibration_width = int(storage.getNode("image_width").real())
    calibration_height = int(storage.getNode("image_height").real())
    storage.release()
    if (K is None or D is None or rectified_K is None or
            calibration_width != width or calibration_height != height):
        raise SystemExit("calibration does not match capture dimensions")
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), rectified_K, (width, height), cv2.CV_16SC2)
os.makedirs(output_dir, exist_ok=True)
capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
capture.set(cv2.CAP_PROP_FPS, fps)
if not capture.isOpened():
    raise SystemExit("cannot open camera: " + device)

def send(kind, payload):
    wire = payload if isinstance(payload, bytes) else payload.encode()
    sys.stdout.buffer.write(kind + struct.pack("!I", len(wire)) + wire)
    sys.stdout.buffer.flush()

prefix = "raw" if raw_mode else "floor"
existing = [name for name in os.listdir(output_dir)
            if name.startswith(prefix + "-") and name.endswith(".png")]
index = 1
for name in existing:
    try:
        index = max(index, int(name[len(prefix) + 1:-4]) + 1)
    except ValueError:
        pass
try:
    while True:
        ok, raw = capture.read()
        if not ok or raw.shape[1] != width or raw.shape[0] != height:
            send(b"E", "camera frame read failed")
            break
        frame = raw if raw_mode else cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if ready:
            command = sys.stdin.readline().strip()
            if command == "capture":
                output = os.path.join(output_dir, "{}-{:03d}.png".format(prefix, index))
                if cv2.imwrite(output, frame):
                    send(b"S", output)
                    index += 1
                else:
                    send(b"E", "cannot save " + output)
            elif command == "quit":
                break
        encoded, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if encoded:
            send(b"F", jpeg.tobytes())
finally:
    capture.release()
"""


def remote_command(args):
    source = shlex.quote(REMOTE_STREAM)
    values = [
        args.device, args.calibration, args.output_dir,
        str(args.width), str(args.height), str(args.fps),
        "1" if args.raw else "0",
    ]
    return "python3 -u -c {} {}".format(source, " ".join(shlex.quote(value) for value in values))


def receive_exact(channel, length):
    data = bytearray()
    while len(data) < length:
        chunk = channel.recv(length - len(data))
        if not chunk:
            raise RuntimeError("camera stream closed")
        data.extend(chunk)
    return bytes(data)


def find_checkerboard(image, full_pattern_size, usable_rows, search_roi_top):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    offset = int(gray.shape[0] * search_roi_top)
    found, corners = cv2.findChessboardCorners(
        gray[offset:, :], full_pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if found:
        corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        corners[:, :, 1] += offset
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
        columns, rows = full_pattern_size
        corners = corners.reshape(rows, columns, 1, 2)[-usable_rows:].reshape(-1, 1, 2)
    return found, corners


def minimum_corner_spacing(corners, pattern_size):
    columns, rows = pattern_size
    grid = corners.reshape(rows, columns, 2)
    horizontal = np.linalg.norm(grid[:, 1:] - grid[:, :-1], axis=2)
    vertical = np.linalg.norm(grid[1:, :] - grid[:-1, :], axis=2)
    return float(min(np.median(horizontal), np.median(vertical)))


def annotate(image, found, corners, pattern_size, captured, spacing, minimum_spacing):
    view = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(view, pattern_size, corners, found)
    valid = found and spacing >= minimum_spacing
    if valid:
        colour = (0, 200, 0)
        message = "BOARD READY - SPACE saves"
    elif found:
        colour = (0, 200, 255)
        message = "BOARD TOO FAR ({:.1f}px) - move closer".format(spacing)
    else:
        colour = (0, 0, 220)
        message = "Move board closer / keep all corners visible"
    cv2.rectangle(view, (12, 12), (760, 60), (0, 0, 0), -1)
    cv2.putText(view, message, (24, 44), cv2.FONT_HERSHEY_SIMPLEX, .65, colour, 2, cv2.LINE_AA)
    cv2.putText(view, "saved: {}   Q/Esc: exit".format(captured), (24, 87),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    if found:
        bounds = corners.reshape(-1, 2)
        centre = np.mean(bounds, axis=0) / np.array([image.shape[1], image.shape[0]])
        area = np.prod(bounds.max(axis=0) - bounds.min(axis=0)) / (image.shape[0] * image.shape[1])
        cv2.putText(view, "coverage: centre=({:.2f}, {:.2f}) area={:.3f}".format(
            centre[0], centre[1], area), (24, 112), cv2.FONT_HERSHEY_SIMPLEX,
            .50, colour, 1, cv2.LINE_AA)
    return view


def main():
    parser = argparse.ArgumentParser(description="Live Cubie camera preview; Space captures a frame.")
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", default="radxa")
    parser.add_argument("--identity-file", type=Path,
                        default=Path.home() / ".ssh" / "id_ed25519_cubie_192_168_19_105")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--calibration",
                        default="/home/radxa/cubie-robot/calibration/camera2_fisheye_1280x720.yaml")
    parser.add_argument("--output-dir", default="/home/radxa/cubie-robot/calibration-session-new/raw")
    parser.add_argument("--raw", action="store_true",
                        help="stream and save raw fisheye frames; use for intrinsic calibration")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--pattern-cols", type=int, default=11)
    parser.add_argument("--pattern-rows", type=int, default=8)
    parser.add_argument("--usable-rows", type=int,
                        help="use only this many bottom checkerboard rows")
    parser.add_argument("--search-roi-top", type=float, default=0.0,
                        help="ignore this fraction of image height at the top")
    parser.add_argument("--min-corner-spacing-px", type=float, default=12.0,
                        help="minimum median spacing required before Space can save")
    args = parser.parse_args()
    usable_rows = args.usable_rows or args.pattern_rows
    if min(args.width, args.height, args.fps, args.pattern_cols, usable_rows,
           args.min_corner_spacing_px) <= 0:
        parser.error("dimensions, fps, and pattern size must be positive")
    if usable_rows > args.pattern_rows or usable_rows < 3:
        parser.error("usable-rows must be from 3 through pattern-rows")
    if not 0 <= args.search_roi_top < 1:
        parser.error("search-roi-top must be in [0, 1)")

    try:
        import paramiko
    except ImportError:
        print("install Paramiko first: python -m pip install paramiko", file=sys.stderr)
        return 1
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        if args.identity_file.is_file():
            client.connect(
                args.host, username=args.user, key_filename=str(args.identity_file), timeout=10,
                auth_timeout=10, banner_timeout=10, look_for_keys=False, allow_agent=False)
        else:
            password = getpass.getpass("Cubie SSH password: ")
            if not password:
                return 1
            client.connect(
                args.host, username=args.user, password=password, timeout=10,
                auth_timeout=10, banner_timeout=10, look_for_keys=False, allow_agent=False)
        channel = client.get_transport().open_session()
        channel.settimeout(10)
        channel.exec_command(remote_command(args))
    except Exception as error:
        client.close()
        print("SSH connection failed: {}".format(error), file=sys.stderr)
        return 1

    full_pattern_size = (args.pattern_cols, args.pattern_rows)
    pattern_size = (args.pattern_cols, usable_rows)
    window = "Cubie checkerboard preview - Space capture, Q exit"
    captured = 0
    print("Preview started in {} mode. Focus the image window; Space saves only a detected board.".format(
        "raw fisheye" if args.raw else "rectified"))
    try:
        while True:
            header = receive_exact(channel, 5)
            kind, length = header[:1], struct.unpack("!I", header[1:])[0]
            payload = receive_exact(channel, length)
            if kind == b"E":
                raise RuntimeError(payload.decode(errors="replace"))
            if kind == b"S":
                captured += 1
                print("saved {}".format(payload.decode(errors="replace")))
                continue
            if kind != b"F":
                raise RuntimeError("unknown stream message")
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            found, corners = find_checkerboard(
                frame, full_pattern_size, usable_rows, args.search_roi_top)
            spacing = minimum_corner_spacing(corners, pattern_size) if found else 0.0
            valid = found and spacing >= args.min_corner_spacing_px
            cv2.imshow(window, annotate(
                frame, found, corners, pattern_size, captured, spacing, args.min_corner_spacing_px))
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q")):
                channel.sendall(b"quit\n")
                break
            if key == ord(" "):
                if valid:
                    channel.sendall(b"capture\n")
                else:
                    print("not captured: board is missing or too far ({:.1f}px)".format(spacing))
    except (RuntimeError, OSError, cv2.error) as error:
        print("preview stopped: {}".format(error), file=sys.stderr)
        return 1
    finally:
        cv2.destroyAllWindows()
        channel.close()
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
