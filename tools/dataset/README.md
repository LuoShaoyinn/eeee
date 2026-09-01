# Dataset Tooling

The ignored `dataset/` workspace uses matching filename stems:

- `images/<stem>.jpg`: source frame.
- `semantic_masks/<stem>.png`: `uint8` semantic mask: `0=other`, `1=white_ground`, `2=blue_fence`.
- `detection_labels/<stem>.txt`: YOLO boxes: `0=red_cube`, `1=yellow_cylinder`, `2=opponent_robot`.

The tools require Python OpenCV. An isolated development environment is:

```sh
uv venv .cache/dataset-venv
uv pip install --python .cache/dataset-venv/bin/python opencv-python
```

Extract two frames per second from the calibrated AVI files:

```sh
.cache/dataset-venv/bin/python tools/dataset/extract_frames.py dataset/raw/*.avi --fps 2
```

Generate HSV-based initial labels, then review them visually:

```sh
.cache/dataset-venv/bin/python tools/dataset/hsv_autolabel.py
.cache/dataset-venv/bin/python tools/dataset/review_labels.py
```

Reviewer controls: `0`/`1`/`2` choose semantic mask class; paint with left mouse and erase with right mouse. `r`, `y`, and `o` select red-cube, yellow-cylinder, and opponent-robot box classes; drag with the left mouse to add a box. Press `x` and click a box to delete it. `[`/`]` move between images and save; `s` saves; `q` saves and exits.
