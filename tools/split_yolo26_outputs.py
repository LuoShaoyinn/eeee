#!/usr/bin/env python3
"""Expose YOLO26 boxes and class scores as independently quantized outputs."""

from __future__ import annotations

import argparse

import onnx
from onnx import TensorProto, helper, shape_inference


BOXES = "/model.23/Mul_2_output_0"
SCORES = "/model.23/Sigmoid_output_0"


def output_shape(model: onnx.ModelProto, tensor_name: str) -> list[int]:
    """Return the inferred static shape of a tensor in an exported graph."""
    inferred = shape_inference.infer_shapes(model)
    values = {value.name: value for value in (*inferred.graph.value_info, *inferred.graph.output)}
    value = values.get(tensor_name)
    if value is None:
        raise ValueError(f"Could not infer shape for {tensor_name}")
    dimensions = [dimension.dim_value for dimension in value.type.tensor_type.shape.dim]
    if len(dimensions) != 3 or dimensions[1] != 4 or not all(dimensions[1:]):
        raise ValueError(f"Unexpected YOLO26 tensor shape for {tensor_name}: {dimensions}")
    # ONNX shape inference leaves the batch dimension unknown for decoded boxes.
    # The static export is always a single-image batch.
    dimensions[0] = 1
    return dimensions


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
    shape = output_shape(model, SCORES)

    del graph.output[:]
    graph.output.extend(
        [
            helper.make_tensor_value_info("boxes", TensorProto.FLOAT, shape),
            helper.make_tensor_value_info("scores", TensorProto.FLOAT, shape),
        ]
    )
    graph.output[0].name = BOXES
    graph.output[1].name = SCORES
    onnx.checker.check_model(model)
    onnx.save(model, args.output)


if __name__ == "__main__":
    main()
