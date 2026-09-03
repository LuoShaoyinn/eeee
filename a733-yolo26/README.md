# A733 YOLO26 Decoder

This is the A733 VIPLite runtime wrapper plus a YOLO26 decoder for the four
classes used by `tag-data`:

1. `other_robot`
2. `red_cube`
3. `yellow_cylinder`
4. `home`

The decoder expects one planar `8400 x 8` float output: four `xywh` box
channels followed by four class-score channels. It reverses the 640px
letterbox transform and applies NMS per class.

The source can be built in an Allwinner model-zoo checkout for A733, or natively
on the Cubie with its existing VIPLite libraries:

```sh
cmake -S a733-yolo26 -B build \
  -DMODEL_ZOO_HOME_DIR=/path/to/model-zoo -DTARGET_NAME=A733
cmake --build build --parallel
LD_LIBRARY_PATH=/path/to/viplite ./build/yolo26_demo_a733 -nb model.nb -i image.jpg
```

## Deployment Result: Incorrect Model Output

On 2026-09-03, this decoder was built natively on the Cubie A7S and executed
`/home/radxa/yolo26n-a733/official_yolo26n_4_pcq_a733.nb` successfully. The
VIPLite runtime reported a single `8400 x 8` float output and NPU inference
took about 14 ms per frame.

The first four planar channels contain valid box-scale values. All four class
score channels are zero for both `dog.jpg` and a labeled arena frame. The same
zero-score behavior is present in the corresponding `official_yolo26n_heads.onnx`
export. Consequently, the decoder produces zero detections; this is the
correct behavior for the supplied converted model, not a post-processing
threshold issue.

The conversion/export must preserve the trained class heads before this model
can be used for robot detections.
