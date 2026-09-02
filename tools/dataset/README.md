# Dataset Tooling

The ignored `dataset/` workspace uses matching filename stems:

- `images/<stem>.jpg`: source frame.
- `semantic_masks/<stem>.png`: `uint8` semantic mask: `0=other`, `1=blue_fence`, `2=white_ground`, `3=home_black`.
- `detection_labels/<stem>.txt`: YOLO boxes: `0=other_robot`, `1=red_cube`, `2=yellow_cylinder`, `3=home`.

The tools use `uv` to provide Python OpenCV:

```sh
uv sync
```

Extract two frames per second from the calibrated AVI files. Re-running this command
overwrites frames with matching stems, so use the dated glob when adding a new batch:

```sh
uv run python tools/dataset/extract_frames.py dataset/videos/*.avi --fps 2
uv run python tools/dataset/extract_frames.py dataset/videos/capture-20260901-*.avi --fps 2
```

Generate HSV-based initial labels, then review them visually:

```sh
uv run python tools/dataset/hsv_tune.py --image dataset/images/<frame>.jpg
uv run python tools/dataset/hsv_autolabel.py
uv run python tools/dataset/review_labels.py
```

`hsv_tune.py` opens trackbars for the selected color range. Press `1` for blue fence, `2` for white ground, `3` for black home, `4`/`5` for the two hue-wrapped red ranges, and `6` for yellow. Press `s` to save `dataset/hsv_thresholds.json`; the batch tagger loads this profile automatically.

The batch tagger labels only blue fence by default. After tuning a class, add it explicitly, for example: `uv run python tools/dataset/hsv_autolabel.py --classes blue_fence white_ground home_black`.

To inspect fence geometry, draw the HSV-segmented upper edge in green and lower edge in orange. The fitter uses Hough line segments from each contour independently, retains up to three long fence panels, then bridges an occlusion with the corresponding straight panel:

```sh
uv run python tools/dataset/fit_fence_edges.py --segments 3
```

Pass `--kernel-output dataset/fence_kernel` to inspect the HSV contour measurements before fitting: green marks the upper contour and red marks the lower contour. This diagnostic is useful for finding blue robot details that connect to the fence mask.

`estimate_fence_pose.py` is an offline geometry experiment for forward-facing
frames where the far `x=3 m` wall and both adjacent corners are visible. It
pairs each top and lower fitted-edge join as one 3D vertical fence corner, using
the known `0.254 m` fence height. The camera height and pitch are held at their
existing calibration values; only field `x`, field `y`, and yaw are optimized.
The corner solution seeds a joint fit: cyan corner-pair residuals remain the
dominant term, while the middle upper/lower Hough panel lines provide a soft
alignment term. `--line-alignment-weight` tunes that term; `0.2` is a
conservative default, while `0` reproduces the original corner-only pose.

```sh
uv run python tools/dataset/estimate_fence_pose.py \
  dataset/images/capture-20260831-100146-0001_00004.jpg \
  --output dataset/fence_pose/capture-20260831-100146-0001_00004.jpg
```

The output draws raw upper/lower contour support in dark green/orange, the
initial Hough corner pairs in cyan, and the final rigid projection as green
(upper), red (lower), and blue (vertical corners). Treat a high
`joint_corner_rms_px` or `joint_line_rms_px` as a rejected visual measurement
rather than updating the robot pose. The command also reports the perpendicular
and forward-ray distance to the `B-D` wall; the forward ray must land within
its `0..1.985 m` span.

The same diagnostic runs a directed dark-seam kernel along each projected
vertical `B`/`D` corner. It compares centre pixels with blue pixels on both
sides and requires continuity along the corner. Magenta means the seam passed
its confidence gate; yellow is a rejected candidate. Dark seams are diagnostic
only until both corners are repeatably accepted across unobstructed frames.

Reviewer controls: `0`/`1`/`2`/`3` choose semantic mask class; paint with left mouse and erase with right mouse. `r`, `y`, `o`, and `h` select red-cube, yellow-cylinder, other-robot, and home box classes; drag with the left mouse to add a box. Press `x` and click a box to delete it. `[`/`]` move between images and save; `s` saves; `q` saves and exits.

For manual object annotation without changing semantic masks, use the focused box reviewer.
It writes only YOLO detection labels. Press `r`, `y`, `o`, or `h` for a red cube,
yellow cylinder, other robot, or home, then drag its tight bounding box. `x` deletes
a box under the cursor.

```sh
uv run python tools/dataset/review_labels.py --detection-only --include 'capture-20260901-*.jpg'
```

For model-assisted review, candidates live separately from verified labels. The
reviewer always shows every image: it loads candidates only for stems outside the
review manifest, then saves accepted labels and appends the stem to that manifest.

```sh
uv run python tools/dataset/review_labels.py --detection-only \
  --proposals dataset/detection_candidates \
  --reviewed-manifest dataset/reviewed_images.txt
```

Build a reproducible YOLO train/validation split from verified labels only. Empty
reviewed label files are retained as negative examples; unreviewed frames are not
treated as negatives.

```sh
uv run python tools/dataset/prepare_yolo_dataset.py --output dataset/yolo
```

`fence_ekf_sim.py` is a deterministic offline test of the intended localization
architecture: noisy body-frame odometry/gyro prediction, then intermittent
partial fence-wall updates. It writes an image to `/tmp/fence-ekf-simulation.png`.

```sh
uv run python tools/dataset/fence_ekf_sim.py
```
