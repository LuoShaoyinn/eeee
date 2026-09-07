# Windows Checkpoint-Assisted Tagger

This folder is designed to be unpacked beside a `dataset/` folder. The dataset
is intentionally separate from Git because it contains source videos and labels.

## Setup

1. Install 64-bit Python 3.10 to 3.13 from python.org and select **Add Python to PATH**.
2. Open Command Prompt in this project folder.
3. Run `tools\windows_tagger\setup_windows.bat` once.
4. Run:

   ```bat
   tools\windows_tagger\run_tagger.bat dataset dataset\models\yolo26n_810_best.pt
   ```

The first run creates `dataset\detection_candidates` from the checkpoint, then
opens the reviewer. Existing verified labels in `dataset\detection_labels` are
never overwritten. Frames are marked reviewed only when saved or when moving
with `[` or `]`.

Reviewer controls: `o`, `r`, `y`, `h` select other robot, red cube, yellow
cylinder, or home; drag with the left mouse to add a box; `x` then click a box
to delete it; `[` and `]` save and move; `s` saves; `q` saves and exits.

To regenerate proposals after changing the checkpoint or confidence threshold:

```bat
tools\windows_tagger\run_tagger.bat dataset dataset\models\yolo26n_810_best.pt --refresh --confidence 0.15
```

The checkpoint labels are fixed as `0=other_robot`, `1=red_cube`,
`2=yellow_cylinder`, and `3=home`. The proposal script only merges highly
overlapping boxes from the same class; it does not merge different classes.
