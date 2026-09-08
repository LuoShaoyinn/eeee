#!/usr/bin/env python3
"""Annotate an image with the original floating-point YOLO26 ONNX model."""

from __future__ import annotations

import argparse
import os
import tempfile

import cv2
import numpy as np
import onnx
import onnxruntime as ort


CLASSES = ["other_robot", "red_cube", "yellow_cylinder", "home"]
INPUT_ROWS = 384
INPUT_COLS = 640
SCORE_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45


def prepare(image: np.ndarray) -> tuple[np.ndarray, float, float, float]:
    height, width = image.shape[:2]
    scale = min(INPUT_ROWS / height, INPUT_COLS / width)
    resized_width = round(width * scale)
    resized_height = round(height * scale)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (resized_width, resized_height))
    pad_x = (INPUT_COLS - resized_width) / 2
    pad_y = (INPUT_ROWS - resized_height) / 2
    padded = cv2.copyMakeBorder(
        resized,
        round(pad_y - 0.1),
        round(pad_y + 0.1),
        round(pad_x - 0.1),
        round(pad_x + 0.1),
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    tensor = np.transpose(padded, (2, 0, 1))[None].astype(np.float32) / 255
    return tensor, scale, pad_x, pad_y


def create_session(model_path: str) -> ort.InferenceSession:
    try:
        return ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    except Exception:
        model = onnx.load(model_path)
        from onnx.version_converter import convert_version

        converted = tempfile.NamedTemporaryFile(suffix=".onnx", delete=False)
        converted.close()
        onnx.save(convert_version(model, 15), converted.name)
        try:
            return ort.InferenceSession(converted.name, providers=["CPUExecutionProvider"])
        finally:
            os.unlink(converted.name)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model")
    parser.add_argument("image")
    parser.add_argument("output")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise ValueError(f"Could not read {args.image}")
    tensor, scale, pad_x, pad_y = prepare(image)
    session = create_session(args.model)
    outputs = session.run(None, {session.get_inputs()[0].name: tensor})
    # The deployment graph exposes box coordinates and class scores as separate
    # tensors so their INT8 quantization scales remain independent.  Accept
    # that form as well as the original combined [1, 8, candidates] graph.
    if len(outputs) == 2:
        boxes_output, scores_output = (item[0] for item in outputs)
    else:
        combined = outputs[0][0]
        boxes_output, scores_output = combined[:4], combined[4:]

    annotated = image.copy()
    total = 0
    for class_id, class_name in enumerate(CLASSES):
        class_scores = scores_output[class_id]
        indexes = np.flatnonzero(class_scores >= SCORE_THRESHOLD)
        boxes = []
        scores = []
        for index in indexes:
            center_x, center_y, width, height = boxes_output[:, index]
            x = int(np.floor((center_x - width / 2 - pad_x) / scale))
            y = int(np.floor((center_y - height / 2 - pad_y) / scale))
            right = int(np.ceil((center_x + width / 2 - pad_x) / scale))
            bottom = int(np.ceil((center_y + height / 2 - pad_y) / scale))
            x = max(0, min(x, image.shape[1] - 1))
            y = max(0, min(y, image.shape[0] - 1))
            right = max(0, min(right, image.shape[1]))
            bottom = max(0, min(bottom, image.shape[0]))
            if right > x and bottom > y:
                boxes.append([x, y, right - x, bottom - y])
                scores.append(float(class_scores[index]))
        kept = cv2.dnn.NMSBoxes(boxes, scores, SCORE_THRESHOLD, NMS_THRESHOLD)
        for kept_index in kept:
            index = int(kept_index)
            x, y, width, height = boxes[index]
            score = scores[index]
            cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 180, 0), 2)
            cv2.putText(
                annotated,
                f"{class_name} {score * 100:.1f}%",
                (x, max(16, y - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 180, 0),
                2,
            )
            total += 1
    cv2.imwrite(args.output, annotated)
    print(f"detections={total}")


if __name__ == "__main__":
    main()
