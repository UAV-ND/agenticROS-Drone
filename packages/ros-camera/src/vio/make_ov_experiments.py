#!/usr/bin/env python3
"""
make_ov_experiments.py - build OpenVINS config variants K, L and M from the base config.

  K  all four IMU noise values x10    (is the filter overconfident in the D435i IMU?)
  L  up_msckf_chi2_multipler 5        (is chi2 throwing away good vision updates?)
  M  K + L together

    python3 make_ov_experiments.py
    python3 make_ov_experiments.py --base ~/vio_config/openvins_d435i --out ~/vio_config/experiments \
        --imu-scale 10 --chi2 5 [--force]

Copies the whole base folder, so the relative imu/imucam paths in estimator_config still
resolve, then edits values in place with regexes: comments and the %YAML header survive.
The base folder is never modified.
"""

import argparse
import os
import re
import shutil
import sys

IMU_KEYS = ("accelerometer_noise_density", "accelerometer_random_walk",
            "gyroscope_noise_density", "gyroscope_random_walk")
CHI2_KEYS = ("up_msckf_chi2_multipler", "up_msckf_chi2_multiplier")  # OpenVINS spells it "multipler"
NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


def find_yaml(folder, needle):
    for name in sorted(os.listdir(folder)):
        path = os.path.join(folder, name)
        if name.endswith((".yaml", ".yml")) and os.path.isfile(path):
            with open(path) as f:
                if needle in f.read():
                    return path
    return None


def scale_imu(path, k, log):
    with open(path) as f:
        text = f.read()
    for key in IMU_KEYS:
        pat = re.compile(rf"^(\s*{key}\s*:\s*)({NUM})(.*)$", re.M)
        m = pat.search(text)
        if not m:
            sys.exit(f"{key} not found in {path}")
        old = float(m.group(2))
        new = old * k
        text = text[:m.start()] + f"{m.group(1)}{new:.4e}{m.group(3)}" + text[m.end():]
        log.append(f"{os.path.basename(path)}: {key} {old:.4e} -> {new:.4e}")
    with open(path, "w") as f:
        f.write(text)


def set_chi2(path, value, log):
    with open(path) as f:
        text = f.read()
    for key in CHI2_KEYS:
        pat = re.compile(rf"^(\s*{key}\s*:\s*)({NUM})(.*)$", re.M)
        m = pat.search(text)
        if m:
            text = text[:m.start()] + f"{m.group(1)}{value:g}{m.group(3)}" + text[m.end():]
            log.append(f"{os.path.basename(path)}: {key} {m.group(2)} -> {value:g}")
            break
    else:
        text = text.rstrip("\n") + f"\nup_msckf_chi2_multipler: {value:g}\n"
        log.append(f"{os.path.basename(path)}: up_msckf_chi2_multipler not present, appended = {value:g} "
                   "(confirm it in OpenVINS's startup print)")
    with open(path, "w") as f:
        f.write(text)


def build(base, out, name, imu_scale, chi2, force):
    dst = os.path.join(out, name)
    if os.path.exists(dst):
        if not force:
            sys.exit(f"{dst} already exists; pass --force to rebuild it.")
        shutil.rmtree(dst)
    shutil.copytree(base, dst)
    log = []
    if imu_scale:
        imu = find_yaml(dst, "accelerometer_noise_density")
        if imu is None:
            sys.exit(f"No IMU yaml (with accelerometer_noise_density) in {base}")
        scale_imu(imu, imu_scale, log)
    if chi2:
        est = find_yaml(dst, "relative_config_imu")
        if est is None:
            sys.exit(f"No estimator config (with relative_config_imu) in {base}")
        set_chi2(est, chi2, log)
    with open(os.path.join(dst, "VARIANT.txt"), "w") as f:
        f.write(f"Variant {name}, built from {base}\n" + "\n".join(log) + "\n")
    print(f"{name}: {dst}")
    for line in log:
        print(f"    {line}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default="~/vio_config/openvins_d435i")
    ap.add_argument("--out", default="~/vio_config/experiments")
    ap.add_argument("--imu-scale", type=float, default=10.0)
    ap.add_argument("--chi2", type=float, default=5.0)
    ap.add_argument("--force", action="store_true", help="rebuild variants that already exist")
    args = ap.parse_args()

    base = os.path.expanduser(args.base)
    out = os.path.expanduser(args.out)
    if not os.path.isdir(base):
        sys.exit(f"Base config folder not found: {base}")
    os.makedirs(out, exist_ok=True)

    build(base, out, "K", args.imu_scale, None, args.force)
    build(base, out, "L", None, args.chi2, args.force)
    build(base, out, "M", args.imu_scale, args.chi2, args.force)


if __name__ == "__main__":
    main()
