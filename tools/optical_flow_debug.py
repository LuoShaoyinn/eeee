"""USB-camera optical-flow debugger for a ground-facing camera.

Examples:
  python tools/optical_flow_debug.py --camera 1
  python tools/optical_flow_debug.py --camera 1 --mode view
  python tools/optical_flow_debug.py --camera 1 --height-m 0.20 --focal-px 650 --csv flow.csv

The metric-speed result assumes a level, ground-facing camera. Use camera
calibration values for focal length; otherwise only pixel velocity is reported.
"""

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np

from camera_rectify import DEFAULT_CAMERA1_CALIBRATION, FisheyeRectifier


def parse_args():
    parser = argparse.ArgumentParser(description="Display and log dense optical flow from a USB camera.")
    parser.add_argument("--camera", type=int, default=1, help="OpenCV camera index (Camera1 is 1).")
    parser.add_argument("--mode", choices=("flow", "view"), default="flow",
                        help="flow: calculate and draw optical flow; view: camera preview only.")
    parser.add_argument("--width", type=int, default=1280,
                        help="Requested camera width; use 0 with --height 0 for native mode.")
    parser.add_argument("--height", type=int, default=720,
                        help="Requested camera height; use 0 with --width 0 for native mode.")
    parser.add_argument("--backend", choices=("auto", "dshow", "msmf", "any"), default="auto",
                        help="Windows capture backend. auto tries dshow, msmf, then OpenCV auto mode.")
    parser.add_argument("--exposure", type=float, default=-6.0,
                        help="Manual UVC exposure value; lower (more negative) darkens the image. Use -6 by default.")
    parser.add_argument("--reopen-after", type=int, default=1,
                        help="Reopen the camera after this many consecutive failed reads (default: 1).")
    parser.add_argument("--reopen-delay", type=float, default=2.0,
                        help="Seconds to wait before reopening a failed camera (default: 2.0).")
    parser.add_argument("--startup-frames", type=int, default=3,
                        help="Consecutive frames required before accepting a camera stream (default: 3).")
    parser.add_argument("--startup-timeout", type=float, default=3.0,
                        help="Maximum seconds to validate each backend/resolution combination (default: 3.0).")
    parser.add_argument("--black-level", type=int, default=8,
                        help="Brightness threshold for detecting a dark/invalid camera frame (default: 8).")
    parser.add_argument("--min-bright-ratio", type=float, default=0.01,
                        help="Minimum fraction of pixels brighter than --black-level for a valid frame (default: 0.01).")
    parser.add_argument("--freeze-after", type=int, default=15,
                        help="Reconnect after this many identical raw camera frames; 0 disables frozen-frame detection (default: 15).")
    parser.add_argument("--height-m", type=float, help="Camera height above flat ground, in metres.")
    parser.add_argument("--focal-px", type=float, help="Calibrated camera focal length, in pixels.")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CAMERA1_CALIBRATION,
                        help="Camera1 fisheye calibration NPZ; its saved rectified projection is applied.")
    parser.add_argument("--csv", type=Path, help="Optional CSV log path.")
    return parser.parse_args()


def backend_order(selected: str, retry_after: str | None = None) -> tuple[str, ...]:
    """Prefer the last stable backend; only then try fallback backends."""
    choices = ("dshow", "msmf", "any") if selected == "auto" else (selected,)
    if retry_after not in choices:
        return choices
    return (retry_after,) + tuple(choice for choice in choices if choice != retry_after)


def usable_camera_frame(frame: np.ndarray | None, black_level: int,
                        min_bright_ratio: float) -> bool:
    """Reject empty and almost-black buffers emitted by USB drivers on reset.

    A reset buffer can contain a few hot/noisy pixels, so checking the maximum
    value alone is insufficient.  Require a small but meaningful proportion of
    the image to be brighter than the black threshold instead.
    """
    if frame is None or frame.size == 0:
        return False
    if black_level == 0:
        return True
    if frame.ndim == 3:
        brightness = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        brightness = frame
    bright_ratio = float(np.count_nonzero(brightness > black_level)) / brightness.size
    return bright_ratio >= min_bright_ratio


def same_source_frame(frame: np.ndarray, previous_frame: np.ndarray | None) -> bool:
    """Compare raw camera buffers only; overlays must never affect this test."""
    return previous_frame is not None and frame.shape == previous_frame.shape and np.array_equal(frame, previous_frame)


def open_camera(index: int, width: int, height: int, backend: str, startup_frames: int,
                startup_timeout: float, black_level: int, min_bright_ratio: float,
                retry_after: str | None = None, exposure: float | None = None):
    """Return a verified camera stream and its first usable frame, or ``None``.

    Some Windows camera drivers report a successful open, then provide just one
    frame.  Requiring several consecutive frames detects that state before the
    flow loop begins.  A requested resolution is tried first, then the camera's
    native mode in case the requested mode is unsupported by a backend.
    """
    api_preferences = {
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "any": cv2.CAP_ANY,
    }
    resolutions = ((width, height), (0, 0)) if width and height else ((0, 0),)
    for backend_name in backend_order(backend, retry_after):
        for requested_width, requested_height in resolutions:
            capture = cv2.VideoCapture(index, api_preferences[backend_name])
            if not capture.isOpened():
                capture.release()
                continue
            if requested_width:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, requested_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_height)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if exposure is not None:
                capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DirectShow: manual mode
                changed = capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
                actual = capture.get(cv2.CAP_PROP_EXPOSURE)
                print(f"Exposure request {exposure:g}; driver reports {actual:g}" if changed
                      else "Camera driver did not accept manual exposure; leaving its current exposure mode.")

            consecutive_frames = 0
            last_frame = None
            deadline = time.monotonic() + startup_timeout
            while time.monotonic() < deadline:
                ok, frame = capture.read()
                if ok and usable_camera_frame(frame, black_level, min_bright_ratio):
                    consecutive_frames += 1
                    last_frame = frame
                    if consecutive_frames >= startup_frames:
                        mode = (f"{requested_width}x{requested_height}"
                                if requested_width else "native resolution")
                        return capture, backend_name, mode, last_frame
                else:
                    consecutive_frames = 0
                    time.sleep(0.02)
            capture.release()
    return None


def robust_flow(flow: np.ndarray) -> tuple[float, float, float]:
    """Return median dx/dy and the fraction of pixels with usable motion."""
    magnitude = cv2.magnitude(flow[..., 0], flow[..., 1])
    valid = (magnitude > 0.05) & (magnitude < 30.0)
    if int(valid.sum()) < 100:
        return 0.0, 0.0, 0.0
    return (float(np.median(flow[..., 0][valid])),
            float(np.median(flow[..., 1][valid])),
            float(valid.mean()))


def draw_vectors(frame: np.ndarray, flow: np.ndarray, step: int = 20) -> np.ndarray:
    overlay = frame.copy()
    for y in range(step // 2, frame.shape[0], step):
        for x in range(step // 2, frame.shape[1], step):
            dx, dy = flow[y, x]
            if dx * dx + dy * dy > 0.25:
                cv2.arrowedLine(overlay, (x, y), (int(x + dx), int(y + dy)),
                                (0, 255, 0), 1, tipLength=0.3)
    return overlay


def status_overlay(frame: np.ndarray, message: str, color: tuple[int, int, int]) -> np.ndarray:
    """Keep the last valid image visible while reporting stream state."""
    display = frame.copy()
    cv2.rectangle(display, (0, 0), (display.shape[1], 36), (0, 0, 0), -1)
    cv2.putText(display, message, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2)
    return display


def main():
    args = parse_args()
    if args.height_m is None and args.focal_px is not None:
        raise SystemExit("--focal-px requires --height-m")
    if args.mode == "view" and args.csv:
        raise SystemExit("--csv is only available with --mode flow")
    if args.reopen_after < 1:
        raise SystemExit("--reopen-after must be at least 1")
    if args.reopen_delay < 0:
        raise SystemExit("--reopen-delay cannot be negative")
    if args.width < 0 or args.height < 0 or (args.width == 0) != (args.height == 0):
        raise SystemExit("--width and --height must both be positive, or both be 0 for native mode")
    if args.startup_frames < 1:
        raise SystemExit("--startup-frames must be at least 1")
    if args.startup_timeout <= 0:
        raise SystemExit("--startup-timeout must be positive")
    if not 0 <= args.black_level <= 255:
        raise SystemExit("--black-level must be between 0 and 255")
    if not 0.0 < args.min_bright_ratio <= 1.0:
        raise SystemExit("--min-bright-ratio must be in (0, 1]")
    if args.freeze_after < 0:
        raise SystemExit("--freeze-after cannot be negative")

    opened = open_camera(args.camera, args.width, args.height, args.backend,
                         args.startup_frames, args.startup_timeout, args.black_level,
                         args.min_bright_ratio, exposure=args.exposure)
    if opened is None:
        raise SystemExit(f"Camera {args.camera} did not produce {args.startup_frames} consecutive frames. "
                         "Check that no other app is using it, try --camera 0, or try --backend msmf.")
    capture, active_backend, active_mode, first_frame = opened
    print(f"Using {active_backend} backend at {active_mode}.")
    try:
        rectifier = FisheyeRectifier(args.calibration, (first_frame.shape[1], first_frame.shape[0]))
    except (FileNotFoundError, KeyError, ValueError) as error:
        capture.release()
        raise SystemExit(f"Cannot apply Camera1 calibration: {error}") from error
    first_frame = rectifier.apply(first_frame)
    if args.height_m is not None and args.focal_px is None:
        args.focal_px = float(rectifier.rectified_K[0, 0])
        print(f"Using saved rectified focal length: {args.focal_px:.1f} px.")
    if (args.height_m is None) != (args.focal_px is None):
        capture.release()
        raise SystemExit("--height-m and --focal-px must be supplied together")

    previous_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    previous_time = time.monotonic()
    last_raw_frame = first_frame.copy()
    repeated_raw_frames = 0
    last_display = first_frame.copy()
    window_name = "Camera view" if args.mode == "view" else "Optical flow debug"
    failed_reads = 0

    csv_file = None
    writer = None
    if args.csv:
        csv_file = args.csv.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(["time_s", "fps", "flow_x_px_s", "flow_y_px_s", "valid_ratio",
                         "ground_x_m_s", "ground_y_m_s"])

    if args.mode == "flow":
        print("Flow mode. Press q or Esc to quit. Keep the camera looking at a textured, evenly lit ground plane.")
    else:
        print("View-only mode. Press q or Esc to quit.")
    try:
        while True:
            ok, frame = capture.read()
            now = time.monotonic()
            frame_is_black = (ok and frame is not None and
                              not usable_camera_frame(frame, args.black_level,
                                                       args.min_bright_ratio))
            frame_is_frozen = False
            if ok and frame is not None and not frame_is_black:
                frame = rectifier.apply(frame)
                if same_source_frame(frame, last_raw_frame):
                    repeated_raw_frames += 1
                else:
                    repeated_raw_frames = 0
                frame_is_frozen = args.freeze_after > 0 and repeated_raw_frames >= args.freeze_after
            if not ok or frame_is_black or frame_is_frozen:
                failed_reads += 1
                detail = ("black frame" if frame_is_black else
                          "frozen raw frame" if frame_is_frozen else "frame read failed")
                # Never compare a post-drop frame with a pre-drop frame: doing
                # so creates a false optical-flow spike after USB reconnects.
                previous_gray = None
                previous_time = None
                if failed_reads == 1:
                    print(f"{detail.capitalize()}; retrying...")
                status = f"STREAM LOST ({detail}) - retrying ({failed_reads}/{args.reopen_after})"
                cv2.imshow(window_name, status_overlay(last_display, status, (0, 0, 255)))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if failed_reads >= args.reopen_after:
                    print(f"{failed_reads} consecutive frame reads failed; reopening camera...")
                    capture.release()
                    if args.reopen_delay:
                        time.sleep(args.reopen_delay)
                    opened = open_camera(args.camera, args.width, args.height, args.backend,
                                         args.startup_frames, args.startup_timeout, args.black_level,
                                         args.min_bright_ratio, active_backend, args.exposure)
                    if opened is None:
                        print("Could not restore a stable camera stream; will retry.")
                    else:
                        capture, active_backend, active_mode, first_frame = opened
                        try:
                            first_frame = rectifier.apply(first_frame)
                        except cv2.error as error:
                            capture.release()
                            raise SystemExit(f"Cannot rectify restored camera stream: {error}") from error
                        previous_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
                        previous_time = time.monotonic()
                        last_raw_frame = first_frame.copy()
                        repeated_raw_frames = 0
                        last_display = first_frame.copy()
                        print(f"Camera stream restored using {active_backend} at {active_mode}.")
                    failed_reads = 0
                continue
            if args.mode == "view":
                last_raw_frame = frame.copy()
                last_display = status_overlay(frame, "VIEW ONLY", (0, 255, 0))
                cv2.imshow(window_name, last_display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if previous_gray is None:
                previous_gray, previous_time = gray, now
                last_raw_frame = frame.copy()
                last_display = status_overlay(frame, "STREAM RESYNCING", (0, 255, 255))
                cv2.imshow(window_name, last_display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue
            dt = now - previous_time
            if dt <= 0.0:
                previous_gray, previous_time = gray, now
                continue
            flow = cv2.calcOpticalFlowFarneback(previous_gray, gray, None,
                                                pyr_scale=0.5, levels=3, winsize=21,
                                                iterations=3, poly_n=5, poly_sigma=1.2,
                                                flags=0)
            dx_px, dy_px, valid_ratio = robust_flow(flow)
            vx_px_s, vy_px_s = dx_px / dt, dy_px / dt
            metric_x = metric_y = float("nan")
            if args.height_m is not None:
                metric_x = vx_px_s * args.height_m / args.focal_px
                metric_y = vy_px_s * args.height_m / args.focal_px

            display = draw_vectors(frame, flow)
            text = f"{1.0 / dt:.1f} FPS  flow: ({vx_px_s:+.1f}, {vy_px_s:+.1f}) px/s  quality: {valid_ratio:.2f}"
            cv2.putText(display, text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 0, 255), 2)
            if args.height_m is not None:
                cv2.putText(display, f"ground: ({metric_x:+.3f}, {metric_y:+.3f}) m/s",
                            (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 0, 255), 2)
            last_display = display
            last_raw_frame = frame.copy()
            cv2.imshow(window_name, last_display)

            if writer:
                writer.writerow([time.time(), 1.0 / dt, vx_px_s, vy_px_s, valid_ratio,
                                 metric_x, metric_y])
            previous_gray, previous_time = gray, now
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
