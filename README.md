# Raspberry Pi 4B minimal Linux

This branch builds a 64-bit Raspberry Pi 4B Buildroot image with musl,
BusyBox, root-only login, Dropbear SSH, Ethernet DHCP, Wi-Fi, USB UVC/V4L2
kernel support, PL011 UART support, Broadcom firmware, and CPU-only NCNN.

The kernel is pinned to the Raspberry Pi GitHub release tag
`stable_20260724`; no Raspberry Pi Git history is cloned. The tarball is
cached by Buildroot under `src-buildroot/dl/linux/`.

## Build

```sh
JOBS=8 ./scripts/build-buildroot.sh
```

Outputs are written to `out-buildroot/images/`, including:

- `Image` and `bcm2711-rpi-4-b.dtb`
- `boot.vfat`
- `rootfs.ext2`/`rootfs.ext4`, `rootfs.cpio.gz`, and `rootfs.tar`
- `sdcard.img`, containing a bootable FAT partition and an ext4 root partition

The build generates `sdcard.img` through Buildroot `genimage` and host mtools;
it does not require sudo or access to a physical block device. The post-image
step can also be rerun explicitly in a private namespace:

```sh
unshare -Urnm env \
  PATH="$PWD/out-buildroot/host/bin:$PATH" \
  HOST_DIR="$PWD/out-buildroot/host" \
  BUILD_DIR="$PWD/out-buildroot/build" \
  buildroot-external/board/pi4/post-image.sh "$PWD/out-buildroot/images"
```

## QEMU smoke test

The minimal BusyBox initramfs validates the kernel, UART, init, and basic
userspace without sudo:

```sh
./scripts/build-qemu-smoke.sh
./scripts/run-qemu-smoke.sh
```

## Before flashing

The default root password is `pi4root`; change it in
`buildroot-external/configs/pi4_minimal_defconfig`. Add Wi-Fi credentials to
the Buildroot overlay before rebuilding. Verify the destination device very
carefully before writing `out-buildroot/images/sdcard.img`; no flash script
writes a physical device automatically.

Physical camera, Ethernet, Wi-Fi, UART, and inference validation still needs
a Raspberry Pi 4B and the appropriate hardware.
