#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BR_VERSION=${BR_VERSION:-2026.02}
BR_SRC="$ROOT/src-buildroot"
BR_OUT="$ROOT/out-buildroot"
EXT="$ROOT/buildroot-external"
JOBS=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
HOSTCC=${HOSTCC:-gcc-15}
HOSTCXX=${HOSTCXX:-g++-15}

if [ ! -d "$BR_SRC/.git" ]; then
    git clone --depth=1 --branch "$BR_VERSION" https://gitlab.com/buildroot.org/buildroot.git "$BR_SRC"
fi

# Configure a new output tree once. Re-running a defconfig on an existing
# tree rewrites .config and can make Buildroot revisit many package stages.
# Remove out-buildroot/ when intentionally changing the Buildroot defconfig.
if [ ! -f "$BR_OUT/.config" ]; then
    make -C "$BR_SRC" BR2_EXTERNAL="$EXT" O="$BR_OUT" HOSTCC="$HOSTCC" HOSTCXX="$HOSTCXX" pi4_minimal_defconfig
    # AS_KERNEL cannot infer the version from a custom tarball and defaults to
    # 2.6. The pinned stable_20260724 source is Linux 6.18; retain that ABI
    # baseline for the initial configuration.
    sed -i \
        -e 's/^BR2_TOOLCHAIN_HEADERS_AT_LEAST="[^"]*"/BR2_TOOLCHAIN_HEADERS_AT_LEAST="6.18"/' \
        -e 's/^# BR2_TOOLCHAIN_HEADERS_AT_LEAST_6_18 is not set/BR2_TOOLCHAIN_HEADERS_AT_LEAST_6_18=y/' \
        "$BR_OUT/.config"
fi
make -C "$BR_SRC" BR2_EXTERNAL="$EXT" O="$BR_OUT" HOSTCC="$HOSTCC" HOSTCXX="$HOSTCXX" -j"$JOBS"
echo "Buildroot output: $BR_OUT/images"
