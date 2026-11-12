"""
Live dice detection with the Arducam (or any USB camera), and image capture for new training data.

Usage:
    python detect.py                          # newest models/dice_vN.pt, camera from config.yaml
    python detect.py --weights models/dice_v1.pt --source 1
    python detect.py --save-every 1           # also save a raw frame every second (data collection)
    python detect.py --list-cameras           # show camera numbers, to find the Arducam

Keys (click the video window first):
    q  quit
    s  save the current raw frame
    c  toggle continuous saving (same as --save-every, default every 1 s)

Saved frames go to captures/arducam_<session>_<ms>.jpg. The session part (start time of
this run) lets prepare_dataset.py keep each session wholly in train or wholly in val.
"""
import argparse
import re
import time
from collections import Counter
from pathlib import Path

import cv2
import torch
import yaml
from ultralytics import YOLO

ROOT = Path(__file__).parent


def newest_model():
    models = sorted(ROOT.glob("models/dice_v*.pt"), key=lambda p: int(re.search(r"v(\d+)", p.stem).group(1)))
    if not models:
        raise SystemExit("No models/dice_v*.pt found: train one with train.py or pass --weights")
    return models[-1]


def list_cameras(cam):
    """Open camera 0-5 and report each one's resolution. The Arducam AR0234 is the one at 1920x1200."""
    found = False
    for i in range(6):
        cap = cv2.VideoCapture(i)
        opened = cap.isOpened()
        if opened:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
            ok, _ = cap.read()
            w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            hint = "  <- likely the Arducam" if (w, h) == (1920, 1200) else ""
            print(f"  camera {i}: {w}x{h}{'' if ok else ' (opened, but no frame)'}{hint}")
            found = True
        cap.release()
        if not opened and i >= 1:
            break  # camera numbers are consecutive; stop at the first missing one
    if not found:
        print("  no cameras found (on macOS, check camera permission for your terminal app)")


def main():
    cam = yaml.safe_load(open(ROOT / "config.yaml"))["camera"]
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=Path, default=None, help="default: newest models/dice_vN.pt")
    ap.add_argument("--source", type=int, default=cam["source"], help="camera index: try 0, 1, 2...")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--save-every", type=float, default=0, help="seconds between auto-saved frames (0 = off)")
    ap.add_argument("--list-cameras", action="store_true", help="show available cameras and exit")
    args = ap.parse_args()
    if args.list_cameras:
        return list_cameras(cam)

    weights = args.weights or newest_model()
    model = YOLO(weights)
    device = "mps" if torch.backends.mps.is_available() else 0 if torch.cuda.is_available() else "cpu"
    print(f"Model {weights.name}: {list(model.names.values())}")

    cap = cv2.VideoCapture(args.source)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # needed for full res / fps over USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    cap.set(cv2.CAP_PROP_FPS, cam["fps"])
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.source}. Try another --source number.")
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {args.source}: {w}x{h}")

    out_dir = ROOT / "captures"
    out_dir.mkdir(exist_ok=True)
    session = time.strftime("%Y%m%d-%H%M")
    save_every = args.save_every
    last_save, saved = 0.0, 0
    win = "Dice detection"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print("Running. Click the video window: q quit, s save frame, c toggle continuous saving.")

    while True:
        ok, img = cap.read()
        if not ok:
            print("No frame from camera. Try another --source number.")
            break

        t0 = time.time()
        result = model.predict(img, conf=args.conf, imgsz=640, device=device, verbose=False)[0]
        infer_ms = (time.time() - t0) * 1000
        counts = Counter(model.names[int(c)] for c in result.boxes.cls)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("c"):
            save_every = 0 if save_every else (args.save_every or 1.0)
            print("Continuous saving", f"ON (every {save_every}s)" if save_every else "OFF")
        if key == ord("s") or (save_every and time.time() - last_save >= save_every):
            cv2.imwrite(str(out_dir / f"arducam_{session}_{int(time.time() * 1000)}.jpg"), img)  # raw, no boxes
            last_save, saved = time.time(), saved + 1

        annotated = result.plot()
        status = f"{weights.stem} | {infer_ms:.0f} ms | saved {saved}" + (" | REC" if save_every else "")
        lines = [status] + [f"{n}: {counts[n]}" for n in model.names.values() if counts[n]]
        for i, text in enumerate(lines):
            cv2.putText(annotated, text, (15, 40 + i * 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255) if save_every and i == 0 else (0, 255, 0), 2)
        cv2.imshow(win, annotated)

    cap.release()
    cv2.destroyAllWindows()
    if saved:
        print(f"Saved {saved} frames to {out_dir}/ (session {session})")


if __name__ == "__main__":
    main()
