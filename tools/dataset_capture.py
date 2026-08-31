"""Collect non-duplicated images from a USB camera for YOLO training."""
import argparse
import getpass
import io
import time
from pathlib import Path

import cv2
import numpy as np
import paramiko

from camera_rectify import DEFAULT_CAMERA1_CALIBRATION, FisheyeRectifier


parser = argparse.ArgumentParser()
parser.add_argument("--camera", type=int, default=1)
parser.add_argument("--source", choices=("local", "cubie"), default="local")
parser.add_argument("--host", default="192.168.19.105")
parser.add_argument("--user", default="radxa")
parser.add_argument("--remote", default="/home/radxa/optical_flow/raw_preview.jpg")
parser.add_argument("--output", type=Path, default=Path("dataset/images/rectified"))
parser.add_argument("--interval", type=float, default=1.0, help="Automatic capture interval, seconds; 0 disables it")
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--calibration", type=Path, default=DEFAULT_CAMERA1_CALIBRATION,
                    help="Camera1 fisheye calibration NPZ; saved images are rectified.")
parser.add_argument("--exposure", type=float, default=-6.0,
                    help="Manual UVC exposure value; lower (more negative) darkens the image")
args = parser.parse_args()

args.output.mkdir(parents=True, exist_ok=True)
camera = None
sftp = None
client = None
rectifier = None
if args.source == "local":
    camera = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DirectShow: manual mode
    if camera.set(cv2.CAP_PROP_EXPOSURE, args.exposure):
        print(f"Exposure request {args.exposure:g}; driver reports {camera.get(cv2.CAP_PROP_EXPOSURE):g}")
    else:
        print("Camera driver did not accept manual exposure; leaving its current exposure mode.")
    if not camera.isOpened():
        raise SystemExit(f"Cannot open local camera {args.camera}. Use --source cubie when the camera is connected to Cubie.")
else:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=getpass.getpass("Cubie SSH password: "),
                   timeout=10, look_for_keys=False, allow_agent=False)
    sftp = client.open_sftp()

index = len(list(args.output.glob("*.jpg")))
last_saved = 0.0
print("SPACE: save now | a: toggle auto capture | q/Esc: exit")
try:
    while True:
        if camera is not None:
            ok, frame = camera.read()
        else:
            buffer = io.BytesIO()
            try:
                sftp.getfo(args.remote, buffer)
                frame = cv2.imdecode(np.frombuffer(buffer.getvalue(), np.uint8), cv2.IMREAD_COLOR)
                ok = frame is not None
            except OSError:
                ok = False
        if not ok:
            continue
        if rectifier is None:
            try:
                rectifier = FisheyeRectifier(args.calibration, (frame.shape[1], frame.shape[0]))
            except (FileNotFoundError, KeyError, ValueError) as error:
                raise SystemExit(f"Cannot apply Camera1 calibration: {error}") from error
            print(f"Applying fisheye rectification from {args.calibration}")
        frame = rectifier.apply(frame)
        now = time.monotonic()
        automatic = args.interval > 0 and now - last_saved >= args.interval
        preview = frame.copy()
        cv2.putText(preview, f"saved={index}  auto={'ON' if args.interval > 0 else 'OFF'}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.imshow("YOLO dataset capture", preview)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("a"):
            args.interval = 0 if args.interval > 0 else 1.0
        if key == ord(" ") or automatic:
            filename = args.output / f"frame_{index:06d}.jpg"
            cv2.imwrite(str(filename), frame)
            print(filename)
            index += 1
            last_saved = now
finally:
    if camera is not None:
        camera.release()
    if sftp is not None:
        sftp.close()
        client.close()
    cv2.destroyAllWindows()
