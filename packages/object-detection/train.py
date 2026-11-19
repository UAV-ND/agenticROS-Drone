"""
Step 4: train a new model version on data/dataset/ and compare it with the previous one.

Each run saves models/dice_vN.pt (N = next number) and adds a row to models/history.csv.
Old and new models are both scored on the *current* val set, so the comparison is fair
even though the val set grows over time.

Usage:
    python train.py --note "added 120 Arducam images, outdoor lighting"
    python train.py --from-latest        # fine-tune the newest model instead of starting fresh
    python train.py --epochs 50
"""
import argparse
import csv
import datetime
import re
import shutil
from pathlib import Path

import torch
import yaml
from ultralytics import YOLO

ROOT = Path(__file__).parent
MODELS = ROOT / "models"


def model_versions():
    return sorted((int(re.search(r"dice_v(\d+)", p.stem).group(1)), p) for p in MODELS.glob("dice_v*.pt"))


def score(weights, data, device):
    m = YOLO(weights)
    r = m.val(data=data, device=device, plots=False, verbose=False)
    return r.box.map50, r.box.map, {m.names[i]: r.box.maps[i] for i in m.names}


def main():
    cfg = yaml.safe_load(open(ROOT / "config.yaml"))
    t = cfg["train"]
    ap = argparse.ArgumentParser()
    ap.add_argument("--note", default="", help="what changed in the data, saved to history.csv")
    ap.add_argument("--from-latest", action="store_true", help="start from the newest models/dice_vN.pt")
    ap.add_argument("--epochs", type=int, default=t["epochs"])
    ap.add_argument("--device", default="mps" if torch.backends.mps.is_available()
                    else 0 if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    data = ROOT / "data/dataset/data.yaml"
    if not data.exists():
        raise SystemExit("No data/dataset yet: run export_roboflow.py and prepare_dataset.py first")

    MODELS.mkdir(exist_ok=True)
    versions = model_versions()
    prev = versions[-1][1] if versions else None
    version = (versions[-1][0] if versions else 0) + 1
    name = f"dice_v{version}"

    start = str(prev) if args.from_latest and prev else t["base_model"]
    print(f"Training {name} from {start}")
    model = YOLO(start)
    model.train(
        data=str(data), epochs=args.epochs, imgsz=t["imgsz"], batch=t["batch"], device=args.device,
        project=str(ROOT / "runs"), name=name, exist_ok=True, patience=20, plots=True,
        # Classes are dice colors: keep hue shifts tiny so blue/black and red/yellow don't blur together
        hsv_h=0.015, hsv_s=0.5, hsv_v=0.4,
        fliplr=0.5, flipud=0.5, degrees=15,  # dice look the same at any angle
    )
    out = MODELS / f"{name}.pt"
    shutil.copy(model.trainer.best, out)

    new50, new, per_class = score(out, data, args.device)
    old50 = old = None
    if prev and [n.lower() for n in YOLO(prev).names.values()] == [c.lower() for c in cfg["classes"]]:
        old50, old, _ = score(prev, data, args.device)

    n = {s: len(list((ROOT / f"data/dataset/images/{s}").glob("*.jpg"))) for s in ("train", "val")}
    history = MODELS / "history.csv"
    new_file = not history.exists()
    with open(history, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["version", "date", "train_images", "val_images", "mAP50", "mAP50-95",
                        "prev_mAP50_same_val", *[f"{c}_mAP50-95" for c in cfg["classes"]], "note"])
        w.writerow([name, datetime.date.today(), n["train"], n["val"], f"{new50:.3f}", f"{new:.3f}",
                    "" if old50 is None else f"{old50:.3f}",
                    *[f"{per_class.get(c, 0):.3f}" for c in cfg["classes"]], args.note])

    print(f"\nSaved {out}  (training plots: runs/{name}/)")
    print(f"  {name:>10}: mAP50={new50:.3f}  mAP50-95={new:.3f}")
    if old50 is not None:
        print(f"  {prev.stem:>10}: mAP50={old50:.3f}  mAP50-95={old:.3f}   (same val set)")
        print("  -> new model is " + ("BETTER" if new > old else "NOT better; detect.py can still use "
                                       f"the old one with --weights {prev.relative_to(ROOT)}"))
    for c, v in per_class.items():
        print(f"  {c:>12} mAP50-95={v:.3f}")


if __name__ == "__main__":
    main()
