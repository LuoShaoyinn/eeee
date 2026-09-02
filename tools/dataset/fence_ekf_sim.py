#!/usr/bin/env python3
"""Test an SE(2) EKF with partial known-fence observations."""

import math
from pathlib import Path

import cv2
import numpy as np


FIELD_X, FIELD_Y = 3.0, 1.985


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def predict(state, covariance, body_delta, process_std):
    x, y, yaw = state
    forward, left, yaw_delta = body_delta
    cosine, sine = math.cos(yaw), math.sin(yaw)
    state = np.array([x + cosine * forward - sine * left,
                      y + sine * forward + cosine * left,
                      wrap(yaw + yaw_delta)])
    jacobian = np.array([[1.0, 0.0, -sine * forward - cosine * left],
                         [0.0, 1.0, cosine * forward - sine * left],
                         [0.0, 0.0, 1.0]])
    return state, jacobian @ covariance @ jacobian.T + np.diag(np.square(process_std))


def update(state, covariance, normal, offset, distance_measurement, heading_measurement):
    """Update from one identified wall: normal-distance plus line orientation."""
    expected = np.array([offset - normal @ state[:2], wrap(state[2] - heading_measurement)])
    residual = np.array([distance_measurement - expected[0], wrap(-expected[1])])
    jacobian = np.array([[-normal[0], -normal[1], 0.0], [0.0, 0.0, 1.0]])
    noise = np.diag([.035 ** 2, math.radians(2.0) ** 2])
    innovation = jacobian @ covariance @ jacobian.T + noise
    gain = covariance @ jacobian.T @ np.linalg.inv(innovation)
    state = state + gain @ residual
    state[2] = wrap(state[2])
    covariance = (np.eye(3) - gain @ jacobian) @ covariance
    return state, covariance


def draw_trajectory(path, truth, dead_reckoning, ekf):
    scale, margin = 220, 80
    image = np.full((round(FIELD_Y * scale) + 2 * margin, round(FIELD_X * scale) + 2 * margin, 3), 250,
                    dtype=np.uint8)
    cv2.rectangle(image, (margin, margin),
                  (margin + round(FIELD_X * scale), margin + round(FIELD_Y * scale)), (180, 90, 0), 3)
    def points(states):
        return np.array([[margin + x * scale, margin + (FIELD_Y - y) * scale] for x, y, _ in states], np.int32)
    cv2.polylines(image, [points(truth)], False, (0, 0, 0), 2)
    cv2.polylines(image, [points(dead_reckoning)], False, (0, 0, 255), 2)
    cv2.polylines(image, [points(ekf)], False, (0, 150, 0), 2)
    cv2.putText(image, "black=true  red=odometry  green=EKF", (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                .6, (0, 0, 0), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)


def main():
    rng = np.random.default_rng(42)
    true_state = np.array([.35, .35, 0.0])
    dead_state = true_state.copy()
    ekf_state = true_state + np.array([.12, -.10, math.radians(8.0)])
    covariance = np.diag([.20 ** 2, .20 ** 2, math.radians(12.0) ** 2])
    truth, dead, filtered = [], [], []
    walls = ((np.array([1.0, 0.0]), FIELD_X), (np.array([0.0, 1.0]), FIELD_Y),
             (np.array([-1.0, 0.0]), 0.0), (np.array([0.0, -1.0]), 0.0))
    for step in range(240):
        control = np.array([.012, .004 * math.sin(step / 20), .006 * math.sin(step / 30)])
        true_state, _ = predict(true_state, np.zeros((3, 3)), control, np.zeros(3))
        noisy_control = control + rng.normal(0.0, [.004, .004, math.radians(.5)])
        dead_state, _ = predict(dead_state, np.zeros((3, 3)), noisy_control, np.zeros(3))
        ekf_state, covariance = predict(ekf_state, covariance, noisy_control, [.008, .008, math.radians(.8)])
        # Alternate one and two visible walls; no measurement is available every fifth frame.
        if step % 5:
            visible = (0,) if step % 9 else (0, 1)
            for index in visible:
                normal, offset = walls[index]
                distance = offset - normal @ true_state[:2] + rng.normal(0, .025)
                heading = true_state[2] + rng.normal(0, math.radians(1.4))
                ekf_state, covariance = update(ekf_state, covariance, normal, offset, distance, heading)
        truth.append(true_state.copy())
        dead.append(dead_state.copy())
        filtered.append(ekf_state.copy())
    truth, dead, filtered = map(np.asarray, (truth, dead, filtered))
    dead_rms = np.sqrt(np.mean(np.sum(np.square(dead[:, :2] - truth[:, :2]), axis=1)))
    ekf_rms = np.sqrt(np.mean(np.sum(np.square(filtered[:, :2] - truth[:, :2]), axis=1)))
    output = Path("/tmp/fence-ekf-simulation.png")
    draw_trajectory(output, truth, dead, filtered)
    print("position_rms_m: odometry={:.3f} ekf={:.3f}".format(dead_rms, ekf_rms))
    print("wrote {}".format(output))


if __name__ == "__main__":
    main()
