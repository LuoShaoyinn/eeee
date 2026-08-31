"""Shared rectification support for the calibrated Camera1 fisheye."""

from pathlib import Path

import cv2
import numpy as np


DEFAULT_CAMERA1_CALIBRATION = (
    Path(__file__).resolve().parent.parent /
    "camera1_fisheye_1280x720_rectilinear_f400.npz"
)


class FisheyeRectifier:
    """Apply the saved Camera1 fisheye model at its calibrated resolution."""

    def __init__(self, calibration_path: Path, image_size: tuple[int, int]):
        if not calibration_path.is_file():
            raise FileNotFoundError(f"Calibration file not found: {calibration_path}")
        calibration = np.load(calibration_path)
        calibration_size = tuple(int(value) for value in calibration["image_size"])
        if image_size != calibration_size:
            raise ValueError(
                f"Camera frame is {image_size[0]}x{image_size[1]}, but {calibration_path.name} "
                f"is calibrated for {calibration_size[0]}x{calibration_size[1]}."
            )
        if "rectified_K" not in calibration:
            raise ValueError(f"{calibration_path.name} has no saved rectified projection.")
        self.rectified_K = calibration["rectified_K"].astype(np.float64)
        self.map_x, self.map_y = cv2.fisheye.initUndistortRectifyMap(
            calibration["K"], calibration["D"], np.eye(3), self.rectified_K,
            image_size, cv2.CV_16SC2,
        )

    def apply(self, frame: np.ndarray) -> np.ndarray:
        return cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)
