#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
archive=${MODEL_ZOO_ARCHIVE:-/home/luoshaoyinn/Downloads/allwinner-model-zoo.tar.gz}
cache_dir=${MODEL_ZOO_CACHE:-"$project_dir/.cache/a733-yolo"}
sdk_parent="$cache_dir/model-zoo"
toolchain_archive="$cache_dir/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu.tar.xz"
toolchain_dir="$cache_dir/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu"
release_dir="$project_dir/build/a733-yolov5"
toolchain_url='https://developer.arm.com/-/media/Files/downloads/gnu-a/10.2-2020.11/binrel/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu.tar.xz'
toolchain_md5_url="${toolchain_url}.asc"

if [ ! -f "$archive" ]; then
    echo "Model-zoo archive not found: $archive" >&2
    exit 1
fi

mkdir -p "$cache_dir" "$sdk_parent" "$release_dir"

sdk_root=$(find "$sdk_parent" -mindepth 1 -maxdepth 1 -type d -name 'awnpu_model_zoo-*' -print -quit)
if [ -z "$sdk_root" ]; then
    gzip -t "$archive"
    tar -xzf "$archive" -C "$sdk_parent"
    sdk_root=$(find "$sdk_parent" -mindepth 1 -maxdepth 1 -type d -name 'awnpu_model_zoo-*' -print -quit)
fi

if [ ! -d "$sdk_root" ]; then
    echo "Could not locate the extracted awnpu_model_zoo directory" >&2
    exit 1
fi

opencv_dir="$sdk_root/3rdparty/opencv/opencv-4.9.0-aarch64-linux-sunxi-glibc"
if [ ! -d "$opencv_dir" ]; then
    unzip -q "$sdk_root/3rdparty/opencv/opencv-4.9.0-aarch64-linux-sunxi-glibc.zip" \
        -d "$sdk_root/3rdparty/opencv"
fi

if [ ! -f "$toolchain_archive" ]; then
    curl -L --fail --retry 2 -o "$toolchain_archive" "$toolchain_url"
fi
if [ ! -d "$toolchain_dir" ]; then
    checksum_file="$cache_dir/toolchain.md5"
    curl -L --fail -sS -o "$checksum_file" "$toolchain_md5_url"
    (cd "$cache_dir" && md5sum -c "$(basename "$checksum_file")")
    tar -xJf "$toolchain_archive" -C "$cache_dir"
fi

compiler="$toolchain_dir/bin/aarch64-none-linux-gnu"
build_dir="$cache_dir/build-yolov5"

cmake -S "$sdk_root/examples/yolov5" -B "$build_dir" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER="$compiler-gcc" \
    -DCMAKE_CXX_COMPILER="$compiler-g++" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_EXTERN_TOOLCHAIN=ON \
    -DEXTERN_DEFINE_TARGET=ON \
    -DTARGET_NAME=A733
cmake --build "$build_dir" --parallel

cp "$build_dir/yolov5_demo_a733" "$release_dir/"
cp "$sdk_root/examples/yolov5/model/yolov5s_rt_uint8_a733.nb" "$release_dir/"
cp "$sdk_root/examples/yolov5/model/dog.jpg" "$release_dir/"
cp "$sdk_root/common/npuruntime/lib_linux_aarch64/A733/libNBGlinker.so" "$release_dir/"
cp "$sdk_root/common/npuruntime/lib_linux_aarch64/A733/libVIPhal.so" "$release_dir/"

echo "Built self-contained payload: $release_dir"
