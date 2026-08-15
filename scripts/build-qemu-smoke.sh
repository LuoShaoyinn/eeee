#!/bin/sh
set -eu

# Rootless first-stage image: kernel + static BusyBox initramfs.
# This intentionally validates boot, UART, init, and basic BusyBox commands;
# physical camera/Wi-Fi and production SSH/NCNN are tested in the full image.
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
OUT="$ROOT/out-qemu"
SRC="$ROOT/src-qemu"
JOBS=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
KERNEL_REF=${KERNEL_REF:-rpi-6.12.y}
BUSYBOX_REF=${BUSYBOX_REF:-1_36_1}

need() { command -v "$1" >/dev/null 2>&1 || { echo "missing host tool: $1" >&2; exit 1; }; }
for x in git make bc bison flex cpio gzip fakeroot aarch64-linux-gnu-gcc; do need "$x"; done
mkdir -p "$OUT" "$SRC"

if [ ! -d "$SRC/linux/.git" ]; then
  git clone --depth=1 --branch "$KERNEL_REF" https://github.com/raspberrypi/linux.git "$SRC/linux"
fi
if [ ! -d "$SRC/busybox/.git" ]; then
  git clone --depth=1 --branch "$BUSYBOX_REF" https://git.busybox.net/busybox "$SRC/busybox"
fi

cd "$SRC/linux"
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
cat "$ROOT/configs/kernel.fragment" >> .config
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig
make -j"$JOBS" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image dtbs
cp arch/arm64/boot/Image "$OUT/Image"
cp arch/arm64/boot/dts/broadcom/bcm2711-rpi-4-b.dtb "$OUT/bcm2711-rpi-4-b.dtb"

cd "$SRC/busybox"
if [ ! -f .config ]; then
  make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- defconfig
  sed -i 's/^# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
  sed -i 's/^CONFIG_TC=y/# CONFIG_TC is not set/' .config
  make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig
fi
make -j"$JOBS" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- CONFIG_STATIC=y busybox
rm -rf "$OUT/rootfs"
mkdir -p "$OUT/rootfs"/{bin,dev,etc,proc,sys,tmp,usr/bin}
cp busybox "$OUT/rootfs/bin/busybox"
ln -s busybox "$OUT/rootfs/bin/sh"
ln -s busybox "$OUT/rootfs/bin/uname"
ln -s busybox "$OUT/rootfs/bin/mount"
ln -s ../bin/busybox "$OUT/rootfs/usr/bin/udhcpc"
cat > "$OUT/rootfs/init" <<'EOF'
#!/bin/sh
mount -t devtmpfs devtmpfs /dev
mount -t proc proc /proc
mount -t sysfs sysfs /sys
echo
echo "Raspberry Pi 4B rootless QEMU smoke image"
echo "Kernel: $(uname -a)"
echo "UART/init/BusyBox are working."
echo
exec /bin/sh -i </dev/console >/dev/console 2>&1
EOF
chmod 755 "$OUT/rootfs/init"
fakeroot sh -c "mknod -m 600 '$OUT/rootfs/dev/console' c 5 1; mknod -m 600 '$OUT/rootfs/dev/ttyAMA1' c 204 65; cd '$OUT/rootfs' && find . -print | sort | cpio -o -H newc --owner=0:0 | gzip -9" > "$OUT/initramfs.cpio.gz"
echo "built $OUT/Image and $OUT/initramfs.cpio.gz"
