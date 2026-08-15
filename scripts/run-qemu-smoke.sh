#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
[ -f "$ROOT/out-qemu/Image" ] || { echo "run scripts/build-qemu-smoke.sh first" >&2; exit 1; }
exec qemu-system-aarch64 \
  -M raspi4b -cpu cortex-a72 -m 2G \
  -kernel "$ROOT/out-qemu/Image" \
  -dtb "$ROOT/out-qemu/bcm2711-rpi-4-b.dtb" \
  -initrd "$ROOT/out-qemu/initramfs.cpio.gz" \
  -append 'earlycon=pl011,0xfe201000 console=ttyAMA1,115200 rdinit=/init' \
  -serial mon:stdio -display none
