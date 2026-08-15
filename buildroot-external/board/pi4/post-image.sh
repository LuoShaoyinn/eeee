#!/bin/sh
set -eu

BINARIES_DIR="$1"
BOARD_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
GENIMAGE="${HOST_DIR:-$BINARIES_DIR/../host}/bin/genimage"
TMP_DIR="${BUILD_DIR:-$BINARIES_DIR/../build}/genimage.pi4"
CFG="$TMP_DIR/genimage.cfg"

mkdir -p "$TMP_DIR/root" "$TMP_DIR/work"
rm -f "$CFG"

# Keep the boot configuration synchronized with the board tree even when
# Buildroot is rebuilt incrementally and the rpi-firmware package's copy is
# already up to date.  This also keeps kernel=Image visible to genimage.
cp "$BOARD_DIR/config.txt" "$BINARIES_DIR/rpi-firmware/config.txt"

{
    printf '%s\n' 'image boot.vfat {' '    vfat {' '        files = {'
    for file in "$BINARIES_DIR"/*.dtb "$BINARIES_DIR"/rpi-firmware/*; do
        [ -f "$file" ] || continue
        printf '            "%s",\n' "${file#"$BINARIES_DIR"/}"
    done
    kernel=$(sed -n 's/^kernel=//p' "$BINARIES_DIR/rpi-firmware/config.txt")
    if [ -n "$kernel" ] && [ -f "$BINARIES_DIR/$kernel" ]; then
        printf '            "%s",\n' "$kernel"
    fi
    printf '%s\n' '        }' '    }' '    size = 64M' '}'
    printf '%s\n' 'image sdcard.img {' '    hdimage {' '    }' '    partition boot {'
    printf '%s\n' '        partition-type = 0xC' '        bootable = "true"' '        image = "boot.vfat"' '    }'
    printf '%s\n' '    partition rootfs {' '        partition-type = 0x83' '        image = "rootfs.ext4"' '    }' '}'
} > "$CFG"

exec "$GENIMAGE" \
    --rootpath "$TMP_DIR/root" \
    --tmppath "$TMP_DIR/work" \
    --inputpath "$BINARIES_DIR" \
    --outputpath "$BINARIES_DIR" \
    --config "$CFG"
