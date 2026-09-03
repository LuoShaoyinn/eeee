# A733 YOLO26 Decoder

This is the A733 VIPLite runtime wrapper plus a YOLO26 decoder for the four
classes used by `tag-data`:

1. `other_robot`
2. `red_cube`
3. `yellow_cylinder`
4. `home`

The decoder expects one planar `8400 x 8` float output: four `xywh` box
channels followed by four class-score channels. It reverses the 640px
letterbox transform and applies NMS per class.

The source can be built in an Allwinner model-zoo checkout for A733, or natively
on the Cubie with its existing VIPLite libraries:

```sh
cmake -S a733-yolo26 -B build \
  -DMODEL_ZOO_HOME_DIR=/path/to/model-zoo -DTARGET_NAME=A733
cmake --build build --parallel
LD_LIBRARY_PATH=/path/to/viplite ./build/yolo26_demo_a733 -nb model.nb -i image.jpg
```
