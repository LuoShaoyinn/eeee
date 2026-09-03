#!/usr/bin/env python3
"""Export YOLO26 decoded boxes and sigmoid class scores before top-k selection."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path, help="Fine-tuned YOLO26 checkpoint")
    parser.add_argument("output", type=Path, help="Destination ONNX path")
    parser.add_argument("--height", type=int, default=384)
    parser.add_argument("--width", type=int, default=640)
    args = parser.parse_args()

    if args.height % 32 or args.width % 32:
        raise ValueError("YOLO26 export dimensions must be multiples of 32")

    yolo = YOLO(args.model)
    head = yolo.model.model[-1]
    if not hasattr(head, "one2one_cv2") or not hasattr(head, "one2one_cv3"):
        raise ValueError("Expected an end-to-end YOLO26 detection head")
    del head.one2one_cv2
    del head.one2one_cv3

    exported = Path(
        yolo.export(
            format="onnx",
            imgsz=(args.height, args.width),
            dynamic=False,
            simplify=False,
            opset=13,
        )
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(exported, args.output)


if __name__ == "__main__":
    main()
