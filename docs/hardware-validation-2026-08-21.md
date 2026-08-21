# A733 NPU hardware validation — 2026-08-21

## Scope

Branch `test/yolo-on-cubie`, forked from commit `466028c` on
`test/cubie-hello-world`.

No bootloader, kernel, DTB, firmware, initramfs, system service, or system NPU
file was changed.  The test payload lives under `/home/radxa/yolo-a733` and can
be removed as one directory.

## Inputs

- Board: Radxa Cubie A7S, Debian Bullseye CLI R6
- Kernel: `5.15.147-21-a733`
- Device: `/dev/vipcore`, supplied by the existing `vipcore` module
- Model-zoo archive: `allwinner-model-zoo.tar.gz`, 208,988,911 bytes; `gzip -t`
  passed
- Archive release directory: `awnpu_model_zoo-v0.9.0-20260116-83a67d4b`
- Model: `examples/yolov5/model/yolov5s_rt_uint8_a733.nb`, 5,120,576 bytes
- Runtime: the archive's A733 `libNBGlinker.so` and `libVIPhal.so`, loaded from
  the private application directory
- Compiler: Arm GNU Toolchain 10.2-2020.11; its published MD5 check passed

The system compiler initially produced a binary requiring glibc newer than
Bullseye.  That binary failed before NPU initialization and was replaced with a
GCC 10.2 build whose newest glibc requirement is 2.29.  The board's system
libraries were not upgraded or replaced.

## Command

```sh
cd /home/radxa/yolo-a733
LD_LIBRARY_PATH=. ./yolov5_demo_a733 \
  -nb yolov5s_rt_uint8_a733.nb -i dog.jpg -l 10
```

## Result

The command exited zero.  Relevant output:

```text
VIPLite driver software version 2.0.3.2-AW-2024-08-30
detection num: 3
16:  91%, [ 135,  221,  311,  535], dog
 2:  67%, [ 470,   74,  688,  173], car
 1:  61%, [ 155,  118,  573,  424], bicycle
network: 0, this network run avg inference time=22629 us, total avg cost: 89713 us
destory npu finished.
```

This validates actual A733 NPU execution, not CPU ONNX inference or a mock.
The 22.629 ms network mean corresponds to about 44.2 inferences/s.  The 89.713
ms total is about 11.1 loops/s for this intentionally inefficient file demo.

A second end-to-end run through the checked-in build/deploy scripts also exited
zero and measured 22.511 ms mean NPU inference and 87.116 ms mean whole-demo
loop time.  This confirms the documented clean-cache procedure reproduces the
manual bring-up.

## Camera boundary

The USB camera is currently attached to the x86-64 host, so `/dev/video0` was
not present on the Cubie during this run.  Live Cubie camera-to-NPU throughput
has therefore not yet been validated.  Previous camera measurements showed the
camera itself delivers about 13.49 frames/s despite advertising 30 frames/s;
use MJPEG when it is reattached to the Cubie.

The next implementation should:

1. capture MJPEG from V4L2 and retain only the newest frame;
2. decode and letterbox once into the resident NPU input buffer;
3. run the already-created network;
4. perform score filtering/NMS without rendering;
5. publish the chosen centre point and confidence;
6. benchmark capture, decode/resize, NPU, post-process, and end-to-end latency
   separately.
