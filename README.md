# YOLO on Radxa Cubie A7S

This branch is a reproducible hardware bring-up for the A733 NPU in the Radxa
Cubie A7S.  It deliberately keeps the vendor kernel, DTB, firmware, `vipcore`
driver, and VIPLite userspace runtime intact.

The first hardware milestone uses the official Allwinner model-zoo YOLOv5s
example.  This is preferable to exporting a new PyTorch model before the NPU
runtime/model ABI has been validated.

## Current result

On 2026-08-21 the supplied `yolov5s_rt_uint8_a733.nb` ran successfully on the
Cubie at `192.168.1.112`:

- VIPLite driver software: `2.0.3.2-AW-2024-08-30`
- NPU inference: `22.629 ms` average over 10 runs (about `44.2` inferences/s)
- whole supplied file-demo loop: `89.713 ms` average
- detections on `dog.jpg`: dog 91%, car 67%, bicycle 61%

The whole-loop number is not the camera throughput.  The vendor demo decodes
the same JPEG again during post-processing and writes an annotated PNG on every
iteration.  A live point-only pipeline should keep the NPU network resident,
decode each camera frame once, omit drawing/image writes, and return only the
selected detection centre `(x, y)`.

See [the hardware validation record](docs/hardware-validation-2026-08-21.md)
for the exact evidence and remaining camera work.

## Reproduce

Requirements on the x86-64 build host:

- `/home/luoshaoyinn/Downloads/allwinner-model-zoo.tar.gz`
- CMake, curl, tar, unzip, and OpenSSH
- network access to download Arm GNU Toolchain 10.2 once

Build the Bullseye-compatible AArch64 package:

```sh
./scripts/build-a733-yolov5.sh
```

Deploy into the user's home directory and run ten iterations (SSH/SCP will ask
for the `radxa` password):

```sh
./scripts/deploy-a733-yolov5.sh radxa@192.168.1.112
```

Everything on the board is contained in `/home/radxa/yolo-a733`.  Removal is
therefore reversible and does not alter system libraries:

```sh
ssh radxa@192.168.1.112 'rm -r /home/radxa/yolo-a733'
```

Do not replace `libc`, the kernel driver, or files under `/usr/lib` with the
bundled runtime.  The launcher uses a private `LD_LIBRARY_PATH` instead.

## Thermal-limited video verification

The deployed `verify_a733_yolov5_video.py` samples frames with the board's
OpenCV installation and invokes the NPU demo once per sample. It logs NPU
temperature before and after every inference and stops when the temperature
reaches 70C by default. This is a verification path, not a real-time pipeline:
the vendor demo creates and destroys the NPU network for each frame.

Run it on the board after deployment:

```sh
cd /home/radxa/yolo-a733
./verify_a733_yolov5_video.py /home/radxa/videos/capture.avi \
  --sample-fps 1 --max-frames 30 --max-temp-c 70
```

Frames, NPU output logs, and `temperatures.csv` are written to a timestamped
`video-verify-*` directory. Use a lower temperature limit when the board is in
a warm enclosure.

## Custom YOLO26n model

The custom model must use two NPU outputs, not one combined `1x8x8400` tensor:

- boxes: `1x4x8400`
- sigmoid class scores: `1x4x8400`

Combining the tensors gives coordinates and class probabilities one INT8 output
scale. The coordinate range is hundreds of pixels, so the 0--1 class scores
round to zero. `tools/split_yolo26_outputs.py` exposes the two tensors before
conversion so they are calibrated independently. The board decoder source is
in `src/a733-yolo26/` and expects outputs in that order.

The validated payload is `official_yolo26n_split_pcq_a733.nb`. On the Cubie it
ran in 13.55 ms and detected a positive calibration image. `arena-test.jpg`
is a negative/low-confidence frame (the original ONNX peak score is 0.096),
so zero detections from that image are expected at the 0.35 score threshold.

## YOLO26s training record

An official pretrained YOLO26s model was fine-tuned on the arena dataset on
2026-09-03. The run used 648 training images, 162 validation images, four
classes, 640-pixel inputs, 100 epochs, and two RTX 4090 GPUs. Global batch 64
(32 images per GPU) was used because full-precision global batch 128 exceeded
the 24 GB GPU memory limit.

The best checkpoint was epoch 93:

- precision: 0.94021
- recall: 0.93242
- mAP50: 0.96401
- mAP50-95: 0.67573

The final epoch reached mAP50-95 0.66728. This improves over the YOLO26n run's
best mAP50-95 of 0.65124, but the small model remains the default deployment
candidate: YOLO26s has about 9.5M parameters and 20.7G FLOPs, versus about
2.4M parameters and 5.4G FLOPs for YOLO26n. The training outputs, including
`best.pt`, remain on persistent training storage and are deliberately not
committed to this hardware deployment repository.
