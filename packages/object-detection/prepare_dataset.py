"""
Step 3: turn data/raw/ (from export_roboflow.py) into a train/val split in data/dataset/.

- Splits by capture session, so near-identical frames never land in both train and val.
- The split is stable: a session's side is decided by a hash of its name, so adding new
  sessions never moves old images between train and val.
- Keeps every Nth background (no-dice) image so they don't swamp the labeled ones.

Usage:
    python prepare_dataset.py
"""
import csv
import hashlib
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import yaml

ROOT = Path(__file__).parent


def source_group(file):
    """Which capture session / video an image came from."""
    f = re.sub(r"_[A-Za-z0-9]{6}$", "", file)  # export id suffix
    patterns = (
        r"(arducam_\d{8}-\d{4})_\d+$",  # detect.py captures: one session per run
        r"(.+?_(?:MOV|mp4|mov|MP4))-\d+$",  # frames extracted from a video (Roboflow names them <video>_mp4-0001)
        r"(.+?)_frame_\d+$",
        r"(PXL_\d{8}_\d{3})",           # phone photos: same 10-minute window
    )
    for pat in patterns:
        m = re.match(pat, f)
        if m:
            return m.group(1)
    return f


def is_val(group, fraction):
    return int(hashlib.md5(group.encode()).hexdigest(), 16) % 1000 < fraction * 1000


def main():
    cfg = yaml.safe_load(open(ROOT / "config.yaml"))
    classes, val_fraction = cfg["classes"], cfg["train"]["val_fraction"]
    src, dst = ROOT / "data/raw", ROOT / "data/dataset"

    rows = list(csv.DictReader(open(src / "manifest.csv")))
    labeled = [r for r in rows if r["boxes"] != "0"]
    background = sorted((r for r in rows if r["boxes"] == "0"), key=lambda r: r["file"])
    background = background[::cfg["train"]["background_every"]]

    sessions = {source_group(r["file"]) for r in rows}
    if len(sessions) < 10:
        # Too few sessions to hold whole ones out: send the last frames of every session to val.
        # Neighboring frames are similar, so scores read a bit high until more sessions exist.
        print(f"Only {len(sessions)} sessions: using the last {val_fraction:.0%} of each session as val")
        split = {}
        for g in sessions:
            items = sorted((r for r in labeled + background if source_group(r["file"]) == g),
                           key=lambda r: r["file"])
            for i, r in enumerate(items):
                split[r["file"]] = "val" if i >= int(len(items) * (1 - val_fraction)) else "train"
    else:
        split = {r["file"]: "val" if is_val(source_group(r["file"]), val_fraction) else "train"
                 for r in labeled + background}

    # Fallbacks for small datasets. A class with no val examples (e.g. all of one color came from a
    # single session that hashed to train): send the last frames of its biggest session to val.
    def class_counts(side):
        return Counter(int(l.split()[0]) for r in labeled if split[r["file"]] == side
                       for l in open(src / "labels" / f"{r['file']}.txt") if l.strip())

    by_group = defaultdict(list)
    for r in labeled:
        by_group[source_group(r["file"])].append(r)
    in_val, in_train = class_counts("val"), class_counts("train")
    for c in in_train:
        if in_val[c] == 0:
            g = max(by_group, key=lambda g: sum(l.startswith(f"{c} ") for r in by_group[g]
                                                for l in open(src / "labels" / f"{r['file']}.txt")))
            items = sorted(by_group[g], key=lambda r: r["file"])
            for r in items[int(len(items) * (1 - val_fraction)):]:
                split[r["file"]] = "val"
            print(f"{classes[c]}: no val examples -> last {val_fraction:.0%} of session {g} moved to val")

    # Same for backgrounds: big background-only sessions in train still give val a tail chunk
    bg_by_group = defaultdict(list)
    for r in background:
        bg_by_group[source_group(r["file"])].append(r)
    for g, items in bg_by_group.items():
        if split[items[0]["file"]] == "train" and len(items) >= 10:
            for r in items[int(len(items) * (1 - val_fraction)):]:
                split[r["file"]] = "val"

    shutil.rmtree(dst, ignore_errors=True)
    for f, s in split.items():
        for kind, ext in (("images", ".jpg"), ("labels", ".txt")):
            (dst / kind / s).mkdir(parents=True, exist_ok=True)
            shutil.copy(src / kind / f"{f}{ext}", dst / kind / s / f"{f}{ext}")

    yaml.safe_dump({"path": str(dst.resolve()), "train": "images/train", "val": "images/val",
                    "names": dict(enumerate(classes))}, open(dst / "data.yaml", "w"), sort_keys=False)

    is_bg = {r["file"] for r in background}
    for s in ("train", "val"):
        files = [f for f, v in split.items() if v == s]
        counts = class_counts(s)
        print(f"{s:>5}: {len(files):4} images ({sum(f not in is_bg for f in files)} labeled, "
              f"{sum(f in is_bg for f in files)} background)  "
              + "  ".join(f"{n}={counts[i]}" for i, n in enumerate(classes)))


if __name__ == "__main__":
    main()
