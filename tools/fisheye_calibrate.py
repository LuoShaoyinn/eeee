"""Capture checkerboard images and calibrate a fisheye camera with OpenCV.

Examples:
  python tools/fisheye_calibrate.py capture --camera 0 --board-cols 9 --board-rows 6 --images calibration_images
  python tools/fisheye_calibrate.py calibrate --board-cols 9 --board-rows 6 --square-size-m 0.020 --images calibration_images

Board dimensions are INNER-corner counts. A 10-by-7 square board has 9-by-6
inner corners. Use the same camera resolution for calibration and operation.
"""

import argparse
from pathlib import Path
import re
import time

import cv2
import numpy as np


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenCV fisheye calibration utility")
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture", help="Capture checkerboard photos interactively")
    capture.add_argument("--camera", type=int, default=0)
    capture.add_argument("--board-cols", type=int, required=True, help="Horizontal INNER corners")
    capture.add_argument("--board-rows", type=int, required=True, help="Vertical INNER corners")
    capture.add_argument("--width", type=int, default=640)
    capture.add_argument("--height", type=int, default=480)
    capture.add_argument("--backend", choices=("auto", "dshow", "msmf", "any"), default="auto",
                         help="Windows camera backend; auto tries DirectShow, Media Foundation, then OpenCV auto")
    capture.add_argument("--exposure", type=float, default=-6.0,
                         help="Manual UVC exposure value; lower (more negative) darkens the image")
    capture.add_argument("--images", type=Path, default=Path("calibration_images"))

    calibrate = subparsers.add_parser("calibrate", help="Calculate and save fisheye intrinsics")
    calibrate.add_argument("--board-cols", type=int, required=True, help="Horizontal INNER corners")
    calibrate.add_argument("--board-rows", type=int, required=True, help="Vertical INNER corners")
    calibrate.add_argument("--square-size-m", type=float, required=True, help="One square edge length in metres")
    calibrate.add_argument("--images", type=Path, default=Path("calibration_images"))
    calibrate.add_argument("--output", type=Path, default=Path("camera_fisheye.npz"))

    preview = subparsers.add_parser("preview", help="Show a live undistorted camera view")
    preview.add_argument("--camera", type=int, default=0)
    preview.add_argument("--width", type=int, default=1280)
    preview.add_argument("--height", type=int, default=720)
    preview.add_argument("--backend", choices=("auto", "dshow", "msmf", "any"), default="auto")
    preview.add_argument("--exposure", type=float, default=-6.0,
                         help="Manual UVC exposure value; lower (more negative) darkens the image")
    preview.add_argument("--calibration", type=Path, required=True)
    preview.add_argument("--balance", type=float, default=0.0,
                         help="0 crops more for fewer black borders; 1 retains the widest field of view")
    preview.add_argument("--focal-px", type=float,
                         help="Manual rectified focal length in pixels. Use this for lenses wider than 180 degrees.")
    return parser.parse_args()


def find_corners(gray: np.ndarray, board: tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board, flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)


def open_camera(index: int, width: int, height: int, backend: str, exposure: float | None = None):
    """Open a Windows USB camera using a backend that can deliver frames."""
    backends = {
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "any": cv2.CAP_ANY,
    }
    names = ("dshow", "msmf", "any") if backend == "auto" else (backend,)
    for name in names:
        camera = cv2.VideoCapture(index, backends[name])
        if not camera.isOpened():
            camera.release()
            continue
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if exposure is not None:
            camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DirectShow: manual mode
            if camera.set(cv2.CAP_PROP_EXPOSURE, exposure):
                print(f"Exposure request {exposure:g}; driver reports {camera.get(cv2.CAP_PROP_EXPOSURE):g}")
            else:
                print("Camera driver did not accept manual exposure; leaving its current exposure mode.")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            ok, frame = camera.read()
            if ok and frame is not None and frame.size:
                print(f"Using {name} backend at {frame.shape[1]}x{frame.shape[0]}.")
                return camera, frame
        camera.release()
    return None


def capture_images(args: argparse.Namespace) -> None:
    board = (args.board_cols, args.board_rows)
    args.images.mkdir(parents=True, exist_ok=True)
    opened = open_camera(args.camera, args.width, args.height, args.backend, args.exposure)
    if opened is None:
        raise SystemExit(f"Cannot open a working stream from camera {args.camera}. "
                         "Close other camera apps, then try --backend dshow or --backend msmf.")
    camera, first_frame = opened

    saved = len(list(args.images.glob("*.png")))
    print("Move the board across the full image and change its tilt. SPACE saves a detected board; q exits.")
    try:
        while True:
            ok, frame = True, first_frame
            first_frame = None
            if frame is None:
                ok, frame = camera.read()
            if not ok:
                raise SystemExit("Camera frame read failed")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners = find_corners(gray, board)
            preview = frame.copy()
            if corners is not None:
                cv2.drawChessboardCorners(preview, board, corners, True)
                status, color = "DETECTED — press SPACE to save", (0, 255, 0)
            else:
                status, color = "Board not detected", (0, 0, 255)
            cv2.putText(preview, f"{status} | saved: {saved}", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
            cv2.imshow("Fisheye calibration capture", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" ") and corners is not None:
                filename = args.images / f"checkerboard_{saved:03d}.png"
                cv2.imwrite(str(filename), frame)
                print(f"Saved {filename}")
                saved += 1
            elif key in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def calibrate(args: argparse.Namespace) -> None:
    board = (args.board_cols, args.board_rows)
    objp = np.zeros((1, board[0] * board[1], 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2)
    objp *= args.square_size_m
    object_points, image_points, usable_files = [], [], []
    image_size = None

    files = sorted((*args.images.glob("*.png"), *args.images.glob("*.jpg"), *args.images.glob("*.jpeg")))
    for filename in files:
        image = cv2.imread(str(filename))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = find_corners(gray, board)
        if corners is None:
            print(f"Skipped (no corners): {filename}")
            continue
        if image_size is None:
            image_size = gray.shape[::-1]
        elif image_size != gray.shape[::-1]:
            print(f"Skipped (different resolution): {filename}")
            continue
        object_points.append(objp)
        image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
        usable_files.append(filename)

    if len(object_points) < 12:
        raise SystemExit(f"Need at least 12 usable images; found {len(object_points)}")

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    active = list(range(len(object_points)))
    rejected = []
    flags = (cv2.fisheye.CALIB_USE_INTRINSIC_GUESS |
             cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
             cv2.fisheye.CALIB_FIX_SKEW)
    while True:
        selected_object_points = [object_points[i] for i in active]
        selected_image_points = [image_points[i] for i in active]
        focal_guess = min(image_size) / 2.0
        K = np.array([[focal_guess, 0.0, image_size[0] / 2.0],
                      [0.0, focal_guess, image_size[1] / 2.0],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), np.float64) for _ in active]
        tvecs = [np.zeros((1, 1, 3), np.float64) for _ in active]
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                selected_object_points, selected_image_points, image_size, K, D, rvecs, tvecs,
                flags, criteria,
            )
            break
        except cv2.error as error:
            match = re.search(r"input array (\d+)", str(error))
            if match is None or len(active) <= 12:
                raise
            rejected_file = usable_files[active.pop(int(match.group(1)))]
            rejected.append(rejected_file)
            print(f"Skipped ill-conditioned view: {rejected_file}")
    rectified_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, image_size, np.eye(3), balance=0.0, new_size=image_size)
    if not np.isfinite(rectified_K).all() or np.min(np.diag(rectified_K)[:2]) < 10.0:
        print("K =\n", K)
        print("D =\n", D.ravel())
        print("Suggested rectified K =\n", rectified_K)
        raise SystemExit("Calibration rejected: the samples do not constrain the full fisheye field of view. "
                         "Capture the full checkerboard near all four image edges and corners, then recalibrate.")
    np.savez(args.output, K=K, D=D, image_size=np.array(image_size),
             board=np.array(board), square_size_m=args.square_size_m, rms=rms)
    yaml_path = args.output.with_suffix(".yaml")
    storage = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    storage.write("K", K)
    storage.write("D", D)
    storage.write("image_width", image_size[0])
    storage.write("image_height", image_size[1])
    storage.write("rms", rms)
    storage.release()
    print(f"Used {len(active)} images; RMS reprojection error: {rms:.4f} px")
    if rejected:
        print(f"Rejected {len(rejected)} ill-conditioned view(s); originals were retained.")
    print("K =\n", K)
    print("D =\n", D.ravel())
    print(f"Saved: {args.output} and {yaml_path}")


def preview_undistorted(args: argparse.Namespace) -> None:
    if not 0.0 <= args.balance <= 1.0:
        raise SystemExit("--balance must be between 0 and 1")
    calibration = np.load(args.calibration)
    K, D = calibration["K"], calibration["D"]
    calibration_size = tuple(int(x) for x in calibration["image_size"])
    opened = open_camera(args.camera, args.width, args.height, args.backend, args.exposure)
    if opened is None:
        raise SystemExit(f"Cannot open a working stream from camera {args.camera}")
    camera, first_frame = opened
    image_size = (first_frame.shape[1], first_frame.shape[0])
    if image_size != calibration_size:
        camera.release()
        raise SystemExit(f"Camera is running at {image_size[0]}x{image_size[1]}, but calibration is "
                         f"for {calibration_size[0]}x{calibration_size[1]}. Use the calibrated resolution.")
    if args.focal_px is not None:
        if args.focal_px <= 0:
            raise SystemExit("--focal-px must be positive")
        new_K = np.array([[args.focal_px, 0.0, image_size[0] / 2.0],
                          [0.0, args.focal_px, image_size[1] / 2.0],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
    elif "rectified_K" in calibration:
        new_K = calibration["rectified_K"].astype(np.float64)
        print("Using saved rectified projection.")
    else:
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, image_size, np.eye(3), balance=args.balance, new_size=image_size)
    if np.min(np.diag(new_K)[:2]) < 10.0:
        camera.release()
        raise SystemExit("Automatic full-field rectification is not possible for this lens. "
                         "Re-run with --focal-px 400 to crop the outermost fisheye region.")
    map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, image_size, cv2.CV_16SC2)
    print("Undistorted preview. Press q or Esc to quit.")
    try:
        frame = first_frame
        while True:
            corrected = cv2.remap(frame, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT)
            cv2.putText(corrected, f"Undistorted | balance={args.balance:.2f}", (14, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.imshow("Camera1 fisheye undistorted", corrected)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            ok, frame = camera.read()
            if not ok or frame is None:
                raise SystemExit("Camera frame read failed")
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main() -> None:
    args = arguments()
    if args.command == "capture":
        capture_images(args)
    elif args.command == "calibrate":
        calibrate(args)
    else:
        preview_undistorted(args)


if __name__ == "__main__":
    main()
