# Dataset Tooling

The ignored `dataset/` workspace uses matching filename stems:

- `images/<stem>.jpg`: source frame.
- `semantic_masks/<stem>.png`: `uint8` semantic mask: `0=other`, `1=blue_fence`, `2=white_ground`, `3=home_black`.
- `detection_labels/<stem>.txt`: YOLO boxes: `0=other_robot`, `1=red_cube`, `2=yellow_cylinder`, `3=home`.

Install the Python tools with `uv`:

```sh
uv sync
```

Extract frames from calibrated AVI files. Re-running overwrites matching image
stems, so use a dated glob when adding a batch:

```sh
uv run python tools/dataset/extract_frames.py dataset/videos/*.avi --fps 2
uv run python tools/dataset/extract_frames.py dataset/videos/capture-20260901-*.avi --fps 2
```

Tune HSV thresholds and generate initial semantic labels:

```sh
uv run python tools/dataset/hsv_tune.py --image dataset/images/<frame>.jpg
uv run python tools/dataset/hsv_autolabel.py
uv run python tools/dataset/review_labels.py
```

`hsv_tune.py` opens trackbars for the selected colour range. Press `1` for blue
fence, `2` for white ground, `3` for black home, `4`/`5` for the hue-wrapped
red ranges, and `6` for yellow. Press `s` to save
`dataset/hsv_thresholds.json`; the batch tagger loads this profile automatically.

The batch tagger labels only blue fence by default. Add tuned semantic classes
explicitly, for example:

```sh
uv run python tools/dataset/hsv_autolabel.py --classes blue_fence white_ground home_black
```

Reviewer controls: `0`/`1`/`2`/`3` choose semantic-mask class; paint with left
mouse and erase with right mouse. `r`, `y`, `o`, and `h` select red-cube,
yellow-cylinder, other-robot, and home box classes; drag with the left mouse to
add a box. Press `x` and click a box to delete it. `[`/`]` move between images
and save; `s` saves; `q` saves and exits.

For object annotation without modifying semantic masks, use focused box review:

```sh
uv run python tools/dataset/review_labels.py --detection-only --include 'capture-20260901-*.jpg'
```

For model-assisted review, candidates remain separate from verified labels. The
reviewer loads proposals only for images outside the review manifest, then saves
accepted labels and appends the stem to that manifest:

```sh
uv run python tools/dataset/review_labels.py --detection-only \
  --proposals dataset/detection_candidates \
  --reviewed-manifest dataset/reviewed_images.txt
```

Build a reproducible YOLO train/validation split from reviewed labels only.
Empty reviewed label files remain negative examples; unreviewed frames are not
treated as negatives.

```sh
uv run python tools/dataset/prepare_yolo_dataset.py --output dataset/yolo
```
