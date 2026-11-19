# Dice detection

YOLO26 model that detects four dice colors (black, blue, yellow, red), trained on images from the
**Arducam 2.3MP AR0234 global-shutter USB 3.0 camera**.

**Just want to run the detector?** Follow [MANUAL.md](MANUAL.md). This README covers adding data
and training new model versions.

| File | What it does |
|---|---|
| `config.yaml` | All settings: Roboflow project, class list, camera, training |
| `detect.py` | Live detection from the camera, and capturing new training images |
| `export_roboflow.py` | Downloads all labeled images from Roboflow into `data/raw/` (no credits needed) |
| `prepare_dataset.py` | Splits `data/raw/` into train/val in `data/dataset/` |
| `train.py` | Trains the next model version into `models/dice_vN.pt` and compares it to the previous one |
| `models/` | Every model version, plus `history.csv` with scores and notes for each |

## Setup

Install as in [MANUAL.md](MANUAL.md) (steps 2–3), i.e. `pip install -r requirements.txt`.

Get your Roboflow **Private API key** (Roboflow → Settings → API Keys) and save it, without it
showing on screen or in shell history:

```bash
read -s "k?Roboflow API key: " && echo "$k" > .roboflow_key && chmod 600 .roboflow_key
```

`.roboflow_key` is git-ignored. Never commit it. Then set `roboflow.project` in `config.yaml`.

## Adding data and training a new model

**1. Capture images with the Arducam**

```bash
python detect.py --save-every 1
```

This saves a raw frame every second to `captures/` while showing the current model's detections.
Keys: `s` saves one frame, `c` turns continuous saving on/off, `q` quits. Each run is one
*session*; keep a session to one setup (one place, one lighting).

What makes the model better:
- each dice color in several places, under different lighting, at different distances and angles
- dice that are small, far away, touching each other, partly hidden, or at the edge of the frame
- some frames with **no dice** at all (they teach the model what is *not* a die)
- the cases the current model gets wrong (watch the live boxes, press `s`)

**2. Label in Roboflow**

Upload `captures/` to the Roboflow project, label the boxes, and **add the images to the dataset**.
Mark images with no dice as null/background. No need to generate a version.

**3. Export, split, train**

```bash
python export_roboflow.py
python prepare_dataset.py
python train.py --note "what you added, e.g. 150 images outdoors, black dice"
```

Training uses the Mac's GPU (or CUDA, else CPU) and takes about 20–60 min. At the end it shows the
new and previous model's scores **on the same val set**, and adds a row to `models/history.csv`.

**4. Try it**

```bash
python detect.py
```

`detect.py` uses the newest `models/dice_vN.pt`. If the new version is worse, delete it, or pick a
version with `--weights models/dice_v2.pt`.

## How the split works

`prepare_dataset.py` keeps every capture session wholly in train or wholly in val, decided by a hash
of the session name. Frames from one session are near-duplicates, so mixing them would make the
scores look much better than the model really is. The hash makes the split stable: adding new
sessions never moves old images between train and val.

With fewer than 10 sessions, it uses the last 20% of each session as val instead, so every color
appears in val. That is the case right now (9 sessions). Those val images come from the same videos
as train, so expect lower scores in a new place or new lighting. New sessions in new places fix this.

## Notes

- **Class order is fixed.** Class ids inside every model follow `classes` in `config.yaml`. Only
  append new classes at the end, and use the same names in Roboflow (capitalization doesn't
  matter; labels with other names are skipped and listed as a warning).
- `export_roboflow.py` flags images with **auto-labeled boxes** (made by a Roboflow model, not a
  person). Review those in Roboflow before trusting them.
- `models/dice_v1.pt` was trained on the earlier phone-camera dataset (963 images, 339 with dice).
  `models/dice_v2.pt` is the first model trained on Arducam images (Roboflow project
  `new-camera-rtx-cv`, 219 images from 5 videos).
  `models/dice_v3.pt` is the current model: 517 Arducam images from 9 videos, including 176
  background images, many with dice-colored decoys. See `models/history.csv` for scores.
- **Black dice are the weakest class** in `dice_v3` (recall 0.78, mAP50-95 0.81, against 0.94 or
  higher mAP50-95 for the other colors). The most useful new data is black dice next to dark objects.
- Background images must really have no dice. A die in a null image teaches the model to ignore it.
