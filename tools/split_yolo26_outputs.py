#!/usr/bin/env python3
"""Expose YOLO26 boxes and class scores as independently quantized outputs."""

from __future__ import annotations

import argparse

import onnx
from onnx import TensorProto, helper


BOXES = "/model.23/Mul_2_output_0"
SCORES = "/model.23/Sigmoid_output_0"
SHAPE = [1, 4, 8400]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="YOLO26 ONNX with final boxes/classes concat")
    parser.add_argument("output", help="Output ONNX with separate boxes and scores")
    args = parser.parse_args()

    model = onnx.load(args.input)
    graph = model.graph
    final_node = graph.node[-1]
    if final_node.op_type != "Concat" or list(final_node.input) != [BOXES, SCORES]:
        raise ValueError("Unexpected YOLO26 final output graph")

    del graph.output[:]
    graph.output.extend(
        [
            helper.make_tensor_value_info("boxes", TensorProto.FLOAT, SHAPE),
            helper.make_tensor_value_info("scores", TensorProto.FLOAT, SHAPE),
        ]
    )
    graph.output[0].name = BOXES
    graph.output[1].name = SCORES
    onnx.checker.check_model(model)
    onnx.save(model, args.output)


if __name__ == "__main__":
    main()
