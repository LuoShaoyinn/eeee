#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
target=${1:-radxa@192.168.1.112}
release_dir="$project_dir/build/a733-yolov5"
remote_dir=/home/radxa/yolo-a733

for file in yolov5_demo_a733 yolov5s_rt_uint8_a733.nb dog.jpg libNBGlinker.so libVIPhal.so; do
    if [ ! -f "$release_dir/$file" ]; then
        echo "Missing $release_dir/$file; run scripts/build-a733-yolov5.sh first" >&2
        exit 1
    fi
done

ssh "$target" "mkdir -p '$remote_dir'"
scp "$release_dir"/* "$target:$remote_dir/"
ssh "$target" "cd '$remote_dir' && chmod +x yolov5_demo_a733 && \
    LD_LIBRARY_PATH=. ./yolov5_demo_a733 \
    -nb yolov5s_rt_uint8_a733.nb -i dog.jpg -l 10"
