"""
Step 2: pull every labeled image from Roboflow into data/raw/ (YOLO format),
without creating a dataset version, so it costs no Roboflow credits.

Re-running only downloads images that are new; labels are always refreshed
(so fixes made in Roboflow come through), and images removed from the
Roboflow dataset are removed here too.

Usage:
    python export_roboflow.py
    python export_roboflow.py --test     # 5 images only, prints one raw API response

API key: ROBOFLOW_API_KEY env var, or a .roboflow_key file next to this script.

Output (data/raw/):
    images/<name>.jpg
    labels/<name>.txt     YOLO: class cx cy w h (normalized)
    manifest.csv          id, file, split, created, tags, boxes, ai_boxes (model-made boxes)
"""
import argparse
import csv
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import requests
import yaml

API = "https://api.roboflow.com"
ROOT = Path(__file__).parent


def list_image_ids(ws, proj, key):
    """Ids of images in the dataset (a project can also hold many unassigned, unlabeled frames)."""
    ids, offset, total = {}, 0, None
    while total is None or offset < total:
        body = {"limit": 250, "offset": offset, "fields": ["id"], "in_dataset": True}
        r = requests.post(f"{API}/{ws}/{proj}/search", params={"api_key": key}, json=body, timeout=60)
        r.raise_for_status()
        data = r.json()
        total = data["total"]
        results = data.get("results", [])
        if not results:
            break
        for x in results:
            ids[x["id"]] = True
        offset += len(results)
    if len(ids) != total:
        print(f"WARNING: listed {len(ids)} unique images but Roboflow reports {total}")
    return list(ids)


def fetch(ws, proj, key, image_id):
    r = requests.get(f"{API}/{ws}/{proj}/images/{image_id}", params={"api_key": key}, timeout=60)
    r.raise_for_status()
    return r.json()["image"]


def to_yolo(ann, classes, unknown):
    if isinstance(ann, str):
        ann = json.loads(ann)
    w, h = float(ann["width"]), float(ann["height"])
    lines, n_ai = [], 0
    ids = {c.lower(): i for i, c in enumerate(classes)}  # match names regardless of capitalization
    for b in ann.get("boxes", []):  # polygons also carry x/y/width/height (their bounding box)
        n_ai += "confidence" in b  # only model-made (auto-labeled) boxes carry a confidence
        if b["label"].lower() not in ids:
            unknown.add(b["label"])
            continue
        lines.append(f"{ids[b['label'].lower()]} {float(b['x'])/w:.6f} {float(b['y'])/h:.6f} "
                     f"{float(b['width'])/w:.6f} {float(b['height'])/h:.6f}")
    return lines, n_ai


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/raw")
    ap.add_argument("--test", action="store_true", help="export 5 images and print one raw response")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(ROOT / "config.yaml"))
    ws, proj, classes = cfg["roboflow"]["workspace"], cfg["roboflow"]["project"], cfg["classes"]

    key = os.environ.get("ROBOFLOW_API_KEY")
    if not key and (ROOT / ".roboflow_key").exists():
        key = (ROOT / ".roboflow_key").read_text().strip()
    if not key:
        sys.exit("Set ROBOFLOW_API_KEY or put the key in .roboflow_key next to this script")

    out = ROOT / args.out
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)

    print(f"Listing images in {ws}/{proj}...")
    try:
        ids = list_image_ids(ws, proj, key)
    except requests.HTTPError as e:
        code = e.response.status_code
        hint = {401: "the API key is wrong or revoked",
                404: "workspace/project not found, or the API key belongs to a different workspace "
                     "(keys are per workspace: Roboflow > Settings > API Keys in this workspace)"}
        sys.exit(f"Roboflow error {code}: {hint.get(code, e.response.text[:200])}")
    print(f"Found {len(ids)} images")
    if args.test:
        ids = ids[:5]
        first = fetch(ws, proj, key, ids[0])
        print("\nRaw API response for one image (check 'annotation' looks right):")
        print(json.dumps({k: v for k, v in first.items() if k != "embedding"}, indent=2)[:3000])

    unknown, rows, failed, new = set(), [], [], []

    def work(image_id):
        try:
            img = fetch(ws, proj, key, image_id)
            stem = Path(img["name"]).stem + "_" + image_id[:6]  # id suffix avoids name clashes
            jpg = out / "images" / f"{stem}.jpg"
            if not jpg.exists():
                jpg.write_bytes(requests.get(img["urls"]["original"], timeout=60).content)
                if cv2.imread(str(jpg)) is None:  # iPhone HEIC saved under a .jpg name -> convert (macOS sips)
                    subprocess.run(["sips", "-s", "format", "jpeg", str(jpg), "--out", str(jpg)],
                                   check=True, capture_output=True)
                new.append(stem)
            ann = img.get("annotation")
            lines, n_ai = to_yolo(ann, classes, unknown) if ann else ([], 0)
            (out / "labels" / f"{stem}.txt").write_text("\n".join(lines))
            rows.append([image_id, stem, img.get("split", ""), img.get("created", ""),
                         ";".join(img.get("tags", []) or []), len(lines), n_ai])
        except Exception as e:
            failed.append((image_id, repr(e).replace(key, "<key>")))

    with ThreadPoolExecutor(8) as pool:
        for i, _ in enumerate(pool.map(work, ids), 1):
            if i % 100 == 0:
                print(f"  {i}/{len(ids)}")

    # Drop local copies of images that were removed from the Roboflow dataset
    removed = 0
    if not args.test and not failed:
        keep = {r[1] for r in rows}
        for kind, ext in (("images", ".jpg"), ("labels", ".txt")):
            for f in (out / kind).glob(f"*{ext}"):
                if f.stem not in keep:
                    f.unlink()
                    removed += kind == "images"

    with open(out / "manifest.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["id", "file", "split", "created", "tags", "boxes", "ai_boxes"])
        wr.writerows(sorted(rows, key=lambda r: str(r[3])))

    print(f"\nDone: {len(rows)} images in {out}/  ({len(new)} new, {removed} removed, {len(failed)} failed)")
    print(f"Images with no boxes (background): {sum(1 for r in rows if r[5] == 0)}")
    print(f"Images with auto-labeled boxes (check these): {sum(1 for r in rows if r[6] > 0)}")
    if unknown:
        print(f"WARNING: labels not in config.yaml classes were skipped: {sorted(unknown)}")
    for image_id, err in failed[:10]:
        print("  failed", image_id, err)


if __name__ == "__main__":
    main()
