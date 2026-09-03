#!/usr/bin/env python3
"""Create a side-by-side comparison of ONNX and A733 annotated frames."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("frames", nargs="+")
    args = parser.parse_args()

    rows = []
    for frame in args.frames:
        onnx_image = cv2.imread(str(args.directory / f"frame-{frame}-onnx.png"))
        npu_image = cv2.imread(str(args.directory / f"frame-{frame}-npu.png"))
        if onnx_image is None or npu_image is None:
            raise ValueError(f"Missing comparison images for frame {frame}")
        cv2.putText(onnx_image, "Original YOLO26 ONNX", (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 180, 0), 3)
        cv2.putText(npu_image, "A733 INT8 NPU", (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        rows.append(cv2.hconcat([onnx_image, npu_image]))
    if not cv2.imwrite(str(args.output), cv2.vconcat(rows)):
        raise RuntimeError(f"Could not write {args.output}")


if __name__ == "__main__":
    main()
