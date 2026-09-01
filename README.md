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
ssh radxa@192.168.50.1 'rm -r /home/radxa/yolo-a733'
```

Do not replace `libc`, the kernel driver, or files under `/usr/lib` with the
bundled runtime.  The launcher uses a private `LD_LIBRARY_PATH` instead.
