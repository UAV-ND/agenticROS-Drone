#!/usr/bin/env python3
"""
vio_triage.py - quick triage for the D435i + OpenVINS divergence (UAV-ND, G6)

Run on the Orin with ROS 2 Jazzy sourced. Read-only: it never modifies the bag
or the config.

    source /opt/ros/jazzy/setup.bash
    python3 vio_triage.py ~/bags/rs_g5_1102_1958 \
        --config-dir ~/vio_config/openvins_d435i \
        --out ~/vio_runs/triage_1958

Checks (each prints PASS / WARN / FAIL):
  1. OpenVINS extrinsics: does T_imu_cam put the right camera at +x of the left?
  2. Stereo pairing: does every infra1 frame have an infra2 frame at the same stamp?
  3. Image timing: jitter and gaps (camera clock vs host-arrival stamps).
  4. IMU stream: monotonic stamps, duplicates, stair-stepped (copied) samples.
  5. IMU at rest: accel magnitude vs g, measured noise vs the config's noise.
  6. Image quality: brightness, trackable corners, blur, emitter-dot heuristic.

Writes report.txt and a few sample frames (PNG) into --out.
"""

import argparse
import collections
import glob
import math
import os
import sys

import numpy as np
import yaml

try:
    import cv2
except ImportError:  # image-quality checks degrade gracefully without OpenCV
    cv2 = None

G = 9.806  # m/s^2
IMG_T = "sensor_msgs/msg/Image"
IMU_T = "sensor_msgs/msg/Imu"
INFO_T = "sensor_msgs/msg/CameraInfo"


def fmt(v, prec=5):
    return "[" + ", ".join(f"{x:+.{prec}f}" for x in np.ravel(v)) + "]"


class Report:
    def __init__(self):
        self.lines = []
        self.verdicts = []

    def _emit(self, s=""):
        print(s)
        self.lines.append(s)

    def section(self, title):
        self._emit()
        self._emit("=" * 78)
        self._emit(title)
        self._emit("=" * 78)

    def info(self, msg):
        self._emit("  " + msg)

    def verdict(self, level, msg):
        self._emit(f"  [{level}] {msg}")
        self.verdicts.append((level, msg))

    def summary(self):
        self.section("SUMMARY (worst first)")
        order = {"FAIL": 0, "WARN": 1, "PASS": 2}
        for level, msg in sorted(self.verdicts, key=lambda v: order[v[0]]):
            self._emit(f"  [{level}] {msg}")


# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------
def load_cv_yaml(path):
    """OpenVINS/Kalibr configs start with the OpenCV directive '%YAML:1.0',
    which PyYAML rejects. Drop it and parse the rest as plain YAML."""
    with open(path) as f:
        text = "\n".join(line for line in f.read().splitlines()
                         if not line.lstrip().startswith("%YAML"))
    return yaml.safe_load(text)


def find_configs(cfg_dir):
    imucam = imu = None
    for path in sorted(glob.glob(os.path.join(cfg_dir, "*.yaml"))):
        try:
            d = load_cv_yaml(path)
        except Exception:
            continue  # estimator_config may use OpenCV-only tags; not needed here
        if not isinstance(d, dict):
            continue
        if imucam is None and "cam0" in d:
            imucam = (path, d)
        if imu is None and "imu0" in d:
            imu = (path, d)
    return imucam, imu


def check_extrinsics(rep, imucam, baseline, imu_in_cam0):
    rep.section("1. OpenVINS extrinsics  (T_imu_cam = R_CtoI | p_CinI, i.e. camera -> IMU)")
    if imucam is None:
        rep.verdict("WARN", "No imucam yaml (a file with a 'cam0' key) in --config-dir; extrinsic check skipped.")
        return {}
    path, d = imucam
    rep.info(f"file: {path}")
    T, cams = {}, {}
    for name in sorted(k for k in d if str(k).startswith("cam")):
        cam = d[name]
        if not isinstance(cam, dict):
            continue
        if "T_imu_cam" in cam:
            M = np.array(cam["T_imu_cam"], dtype=float).reshape(4, 4)
        elif "T_cam_imu" in cam:
            # OpenVINS's parser falls back to T_cam_imu and inverts it (ov_core opencv_yaml_parse.h).
            M = np.linalg.inv(np.array(cam["T_cam_imu"], dtype=float).reshape(4, 4))
            rep.info(f"{name}: only T_cam_imu present; OpenVINS inverts it, so checking inv(T_cam_imu).")
        else:
            continue
        R, p = M[:3, :3], M[:3, 3]
        T[name], cams[name] = M, cam
        rep.info(f"{name}: topic={cam.get('rostopic')}  res={cam.get('resolution')}  "
                 f"model={cam.get('camera_model')}/{cam.get('distortion_model')}")
        rep.info(f"       intrinsics={cam.get('intrinsics')}  distortion={cam.get('distortion_coeffs')}")
        rep.info(f"       translation column (should be p_CinI) = {fmt(p)} m")
        err = np.abs(R @ R.T - np.eye(3)).max()
        if err > 1e-3 or abs(np.linalg.det(R) - 1.0) > 1e-3:
            rep.verdict("FAIL", f"{name}: rotation block is not a proper rotation (max|RR^T-I| = {err:.1e}).")

    if "cam0" in T:
        R0, p0 = T["cam0"][:3, :3], T["cam0"][:3, 3]
        expected = -R0 @ np.asarray(imu_in_cam0, dtype=float)  # p_CinI = -R_CtoI * p_IinC
        if np.linalg.norm(p0 - expected) < 0.002:
            rep.verdict("PASS", f"cam0 translation matches the driver TF in OpenVINS's convention {fmt(expected)}.")
        elif np.linalg.norm(p0 + expected) < 0.002:
            rep.verdict("WARN", "cam0 translation is the IMU-in-camera vector (wrong direction for T_imu_cam). "
                                "Only mm-level on its own, but it means the convention was misread: see cam1.")
        else:
            rep.verdict("WARN", f"cam0 translation {fmt(p0)} differs from the expected {fmt(expected)} "
                                "(from --imu-in-cam0).")

    if "cam0" in T and "cam1" in T:
        T01 = np.linalg.inv(T["cam0"]) @ T["cam1"]  # pose of cam1 expressed in the cam0 frame
        t = T01[:3, 3]
        cos_a = max(-1.0, min(1.0, (np.trace(T01[:3, :3]) - 1.0) / 2.0))
        ang = math.degrees(math.acos(cos_a))
        rep.info(f"cam1 origin in cam0 frame = {fmt(t)} m, relative rotation {ang:.3f} deg")
        if abs(abs(t[0]) - baseline) > 0.003 or math.hypot(t[1], t[2]) > 0.003:
            rep.verdict("WARN", f"Stereo baseline in config is {fmt(t)}; expected about [+{baseline:.4f}, 0, 0].")
        elif t[0] > 0:
            rep.verdict("PASS", f"Right camera sits at +x = {t[0] * 1000:.2f} mm from the left camera (correct).")
        else:
            rep.verdict("FAIL", f"Right camera sits at x = {t[0] * 1000:.2f} mm: the stereo baseline is INVERTED. "
                                "The matrices look like Kalibr's T_cam_imu (IMU -> camera). Invert both 4x4s "
                                "for OpenVINS's T_imu_cam and rerun the bag.")
        if ang > 0.5:
            rep.verdict("WARN", f"Rectified infra1/infra2 should share orientation; relative rotation is {ang:.2f} deg.")
    return cams


# --------------------------------------------------------------------------
# Bag
# --------------------------------------------------------------------------
def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def open_bag(bag):
    import rosbag2_py
    storage_id = ""
    meta = os.path.join(bag, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as f:
            info = (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information", {})
        storage_id = info.get("storage_identifier", "") or ""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    return reader


def pick_topic(types, override, msg_type, must=(), prefer=()):
    if override:
        if override not in types:
            sys.exit(f"Topic {override} not in bag. Topics: {sorted(types)}")
        return override
    cands = sorted(n for n, t in types.items() if t == msg_type and all(m in n for m in must))
    for p in prefer:
        hit = [c for c in cands if p in c]
        if hit:
            return hit[0]
    return cands[0] if cands else None


def to_gray8(msg):
    enc = msg.encoding.lower()
    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
    except TypeError:
        buf = np.asarray(msg.data, dtype=np.uint8)
    if enc in ("mono8", "8uc1"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
    elif enc in ("mono16", "16uc1"):
        img = (buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width] >> 8).astype(np.uint8)
    else:
        return None
    return np.ascontiguousarray(img)


def image_quality(img):
    q = {"mean": float(img.mean()),
         "sat": float((img >= 250).mean() * 100.0),
         "dark": float((img <= 10).mean() * 100.0)}
    if cv2 is not None:
        pts = cv2.goodFeaturesToTrack(img, maxCorners=1000, qualityLevel=0.01, minDistance=8)
        q["corners"] = 0 if pts is None else int(len(pts))
        q["sharp"] = float(cv2.Laplacian(img, cv2.CV_64F).var())
        # Projector dots = many tiny isolated bright blobs. Heuristic only; eyeball the PNGs.
        tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
        _, bw = cv2.threshold(tophat, 30, 255, cv2.THRESH_BINARY)
        _, _, stats, _ = cv2.connectedComponentsWithStats(bw, connectivity=8)
        q["dots"] = int(np.count_nonzero(stats[1:, cv2.CC_STAT_AREA] <= 12))
    return q


def read_bag(bag, args, rep):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_bag(bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    left = pick_topic(types, args.left, IMG_T, ("infra1",), ("image_rect_raw",))
    right = pick_topic(types, args.right, IMG_T, ("infra2",), ("image_rect_raw",))
    imu_cands = sorted(n for n, t in types.items() if t == IMU_T)
    imu = args.imu or next((n for n in imu_cands if n.rstrip("/").endswith("imu")), None)
    info_l = pick_topic(types, None, INFO_T, ("infra1",))
    info_r = pick_topic(types, None, INFO_T, ("infra2",))

    rep.section("Bag contents")
    rep.info(f"bag: {bag}")
    for n in sorted(types):
        rep.info(f"  {n}  [{types[n]}]")
    rep.info(f"using left={left}  right={right}  imu={imu}")
    if not (left and right and imu):
        rep.verdict("FAIL", "Could not find infra1/infra2 images and a united IMU topic; pass --left/--right/--imu.")
        return None

    wanted = [t for t in (left, right, imu, info_l, info_r) if t]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    cls = {t: get_message(types[t]) for t in wanted}

    d = {"left_ns": [], "right_ns": [], "imu_ns": [], "acc": [], "gyr": [],
         "q_left": [], "q_right": [], "info_l": None, "info_r": None,
         "left": left, "right": right, "imu": imu, "bad_encoding": None}
    counts = {"left": 0, "right": 0}
    targets = [1.0, 15.0, 30.0, 45.0]  # seconds after the first left frame
    next_target, t0 = 0, None
    pending_right = {}                  # stamp -> label, right frames still to save
    recent_right = collections.OrderedDict()
    os.makedirs(args.out, exist_ok=True)

    def save(img, side, label):
        if cv2 is not None and img is not None:
            cv2.imwrite(os.path.join(args.out, f"{label}_{side}.png"), img)

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserialize_message(raw, cls[topic])
        if topic == imu:
            d["imu_ns"].append(stamp_ns(msg.header.stamp))
            a, w = msg.linear_acceleration, msg.angular_velocity
            d["acc"].append((a.x, a.y, a.z))
            d["gyr"].append((w.x, w.y, w.z))
        elif topic in (left, right):
            side = "left" if topic == left else "right"
            ns = stamp_ns(msg.header.stamp)
            d[side + "_ns"].append(ns)
            k = counts[side]
            counts[side] += 1
            label = None
            if side == "left":
                if t0 is None:
                    t0 = ns
                if next_target < len(targets) and (ns - t0) / 1e9 >= targets[next_target]:
                    label = f"t{(ns - t0) / 1e9:05.1f}s"
                    next_target += 1
                    if ns in recent_right:
                        save(to_gray8(recent_right[ns]), "right", label)
                    else:
                        pending_right[ns] = label
            else:
                recent_right[ns] = msg
                while len(recent_right) > 5:
                    recent_right.popitem(last=False)
                label = pending_right.pop(ns, None)
            if k % args.sample_every == 0 or label:
                img = to_gray8(msg)
                if img is None:
                    d["bad_encoding"] = msg.encoding
                    continue
                if k % args.sample_every == 0:
                    d["q_" + side].append(image_quality(img))
                if label:
                    save(img, side, label)
        elif topic == info_l and d["info_l"] is None:
            d["info_l"] = msg
        elif topic == info_r and d["info_r"] is None:
            d["info_r"] = msg
    return d


# --------------------------------------------------------------------------
# Checks on bag data
# --------------------------------------------------------------------------
def check_bag_vs_config(rep, d, cams, imu_cfg):
    rep.section("1b. Bag camera_info and topics vs config")
    for tag, msg, key in (("infra1", d["info_l"], "cam0"), ("infra2", d["info_r"], "cam1")):
        if msg is None:
            rep.info(f"{tag}: no camera_info in bag; skipped.")
            continue
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        P = np.array(msg.p, dtype=float).reshape(3, 4)
        rep.info(f"{tag}: {msg.width}x{msg.height} fx={K[0, 0]:.3f} fy={K[1, 1]:.3f} "
                 f"cx={K[0, 2]:.3f} cy={K[1, 2]:.3f} D={[round(x, 5) for x in msg.d]} P[0,3]={P[0, 3]:.4f}")
        cam = cams.get(key)
        if cam and cam.get("intrinsics"):
            fx, fy, cx, cy = [float(x) for x in cam["intrinsics"]][:4]
            diff = max(abs(fx - K[0, 0]), abs(fy - K[1, 1]), abs(cx - K[0, 2]), abs(cy - K[1, 2]))
            res = cam.get("resolution")
            if diff > 0.5:
                rep.verdict("FAIL", f"{key} intrinsics differ from the bag's {tag} camera_info by {diff:.2f} px.")
            elif res and [int(x) for x in res] != [msg.width, msg.height]:
                rep.verdict("FAIL", f"{key} resolution {res} != bag {msg.width}x{msg.height}.")
            else:
                rep.verdict("PASS", f"{key} intrinsics and resolution match the bag's camera_info.")
    if d["info_r"] is not None:
        P = np.array(d["info_r"].p, dtype=float).reshape(3, 4)
        if P[0, 0] > 0 and P[0, 3] != 0:
            rep.info(f"baseline implied by infra2 projection matrix: {-P[0, 3] / P[0, 0] * 1000:+.2f} mm "
                     "(positive = right camera at +x)")
    for key, topic in (("cam0", d["left"]), ("cam1", d["right"])):
        cfg_topic = cams.get(key, {}).get("rostopic")
        if cfg_topic and cfg_topic != topic:
            rep.verdict("WARN", f"{key} rostopic in config is {cfg_topic} but the bag has {topic} "
                                "(ignore if your launch remaps it).")
    if imu_cfg:
        cfg_topic = (imu_cfg[1].get("imu0") or {}).get("rostopic")
        if cfg_topic and cfg_topic != d["imu"]:
            rep.verdict("WARN", f"imu0 rostopic in config is {cfg_topic} but the bag has {d['imu']} "
                                "(ignore if your launch remaps it).")


def check_stereo(rep, d):
    rep.section("2. Stereo pairing (infra1 vs infra2 header stamps)")
    L = np.sort(np.asarray(d["left_ns"], dtype=np.int64))
    R = np.sort(np.asarray(d["right_ns"], dtype=np.int64))
    if len(L) < 10 or len(R) < 10:
        rep.verdict("FAIL", f"Too few frames (left {len(L)}, right {len(R)}).")
        return
    idx = np.clip(np.searchsorted(R, L), 1, len(R) - 1)
    nearest = np.minimum(np.abs(R[idx] - L), np.abs(R[idx - 1] - L)) / 1e6  # ms
    exact = int((nearest == 0).sum())
    orphans = nearest > 1.0
    n_orphan = int(orphans.sum())
    rep.info(f"left {len(L)}  right {len(R)}  exact stamp matches {exact} ({100 * exact / len(L):.2f}%)  "
             f"left frames with no right frame within 1 ms: {n_orphan}")
    if n_orphan:
        rep.info(f"orphans: nearest right frame is a median {np.median(nearest[orphans]):.1f} ms away")
    elif exact < len(L):
        rep.info(f"pairs are close but not identical: max offset {nearest.max():.3f} ms")
    frac = n_orphan / len(L)
    if frac == 0:
        rep.verdict("PASS", "Every left frame has a right frame at (nearly) the same stamp.")
    elif frac < 0.01:
        rep.verdict("WARN", f"{n_orphan} orphan left frames. OpenVINS's approximate-time sync can pair those "
                            "with a right frame from a different instant.")
    else:
        rep.verdict("FAIL", f"{100 * frac:.1f}% of left frames have no matching right frame. Stereo pairs from "
                            "different instants will wreck triangulation.")


def check_image_timing(rep, d):
    rep.section("3. Image timing")
    for side in ("left", "right"):
        a = np.asarray(d[side + "_ns"], dtype=np.int64)
        if len(a) < 10:
            continue
        back = int((np.diff(a) <= 0).sum())
        dt = np.diff(np.sort(a)) / 1e6
        dt = dt[dt > 0]
        med = float(np.median(dt))
        normal = dt[(dt > 0.5 * med) & (dt < 1.5 * med)]
        jitter = float(normal.std()) if len(normal) else float("nan")
        gaps = int((dt >= 1.5 * med).sum())
        rep.info(f"{side}: {len(a)} frames over {(a.max() - a.min()) / 1e9:.1f} s, ~{1000 / med:.2f} fps, "
                 f"period median {med:.3f} ms, jitter {jitter:.3f} ms, gaps {gaps}, max dt {dt.max():.1f} ms, "
                 f"out-of-order/duplicate {back}")
        if back:
            rep.verdict("FAIL", f"{side}: {back} image stamps out of order or duplicated.")
        if jitter > 1.0:
            rep.verdict("WARN", f"{side}: {jitter:.2f} ms stamp jitter looks like host-arrival time rather than the "
                                "camera clock (RSUSB without frame metadata?). Hurts VIO accuracy.")
        elif side == "left":
            rep.verdict("PASS", f"Image stamps are regular ({jitter:.3f} ms jitter).")


def check_imu_stream(rep, d):
    rep.section("4. IMU stream")
    ns = np.asarray(d["imu_ns"], dtype=np.int64)
    acc = np.asarray(d["acc"], dtype=float)
    gyr = np.asarray(d["gyr"], dtype=float)
    if len(ns) < 100:
        rep.verdict("FAIL", f"Only {len(ns)} IMU messages.")
        return None
    step = np.diff(ns)
    nonmono, dup = int((step < 0).sum()), int((step == 0).sum())
    pos = np.diff(np.unique(ns)) / 1e9
    med = float(np.median(pos))
    rate = 1.0 / med
    rep.info(f"messages {len(ns)}  rate ~{rate:.1f} Hz  dt median {med * 1e3:.3f} ms  "
             f"p99 {np.percentile(pos, 99) * 1e3:.3f} ms  max {pos.max() * 1e3:.1f} ms")
    rep.info(f"out-of-order stamps {nonmono}   duplicate stamps {dup}")
    acc_rep = float(np.all(np.diff(acc, axis=0) == 0, axis=1).mean())
    gyr_rep = float(np.all(np.diff(gyr, axis=0) == 0, axis=1).mean())
    rep.info(f"consecutive identical samples: accel {acc_rep * 100:.1f}%   gyro {gyr_rep * 100:.1f}%")
    if nonmono or dup:
        rep.verdict("FAIL", f"IMU stamps not strictly increasing ({nonmono} backwards, {dup} duplicates). "
                            "OpenVINS propagates with bad dt; check unite_imu_method and the timestamp source.")
    else:
        rep.verdict("PASS", "IMU stamps strictly increasing.")
    big = int((pos > 5 * med).sum())
    if big:
        rep.verdict("WARN", f"{big} IMU gaps longer than 5x nominal (max {pos.max() * 1e3:.1f} ms).")
    if acc_rep > 0.25:
        rep.verdict("WARN", f"{acc_rep * 100:.0f}% of IMU messages repeat the previous accel value (stair-stepped). "
                            "Use unite_imu_method linear_interpolation if it isn't already.")
    return {"rate": rate, "acc_rep": acc_rep, "gyr_rep": gyr_rep}


def check_imu_static(rep, d, stream, imu_cfg):
    rep.section("5. IMU at rest (stillest 2 s window, found automatically)")
    if stream is None:
        return
    ns = np.asarray(d["imu_ns"], dtype=np.int64)
    order = np.argsort(ns, kind="stable")
    ns = ns[order]
    acc = np.asarray(d["acc"], dtype=float)[order]
    gyr = np.asarray(d["gyr"], dtype=float)[order]
    rate = stream["rate"]
    win = max(int(2.0 * rate), 50)
    if len(ns) <= win:
        rep.verdict("WARN", "Bag too short for a 2 s rest window.")
        return
    csum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(gyr, axis=1))])
    means = (csum[win:] - csum[:-win]) / win
    s = int(np.argmin(means))
    rep.info(f"stillest window starts at t = {(ns[s] - ns[0]) / 1e9:.1f} s, mean |gyro| {means[s]:.4f} rad/s")
    if means[s] > 0.03:
        rep.verdict("WARN", "No truly static 2 s window found; the rest statistics below are unreliable.")
    A, W = acc[s:s + win], gyr[s:s + win]
    a_mean = A.mean(axis=0)
    amag = float(np.linalg.norm(a_mean))
    rep.info(f"accel mean {fmt(a_mean, 3)} m/s^2   |a| = {amag:.3f}  (expect ~{G:.3f})")
    rep.info(f"gyro bias  {fmt(W.mean(axis=0), 5)} rad/s")
    err = abs(amag - G)
    if err > 0.25:
        rep.verdict("FAIL", f"|accel| at rest is off by {err:.2f} m/s^2: the IMU is uncalibrated. Run Intel's "
                            "rs-imu-calibration (writes the calibration into the camera) before more VIO tuning.")
    elif err > 0.08:
        rep.verdict("WARN", f"|accel| at rest is off by {err:.2f} m/s^2; consider rs-imu-calibration.")
    else:
        rep.verdict("PASS", f"|accel| at rest within {err:.2f} m/s^2 of g. (Axis misalignment can still hide in a "
                            "single orientation; the 6-pose rs-imu-calibration is the real test.)")

    # Discrete std -> continuous noise density: sigma_c ~= sigma_d / sqrt(sample rate).
    # Rough lower bound: on-sensor filtering lowers sigma_d, and vibration in flight is far worse.
    a_nd = float(A.std(axis=0).mean()) / math.sqrt(rate * max(1e-3, 1.0 - stream["acc_rep"]))
    g_nd = float(W.std(axis=0).mean()) / math.sqrt(rate * max(1e-3, 1.0 - stream["gyr_rep"]))
    rep.info(f"measured at-rest noise density (rough): accel {a_nd:.2e} m/s^2/sqrt(Hz)   gyro {g_nd:.2e} rad/s/sqrt(Hz)")
    if imu_cfg is None:
        rep.info("no imu yaml (with an 'imu0' key) in --config-dir; comparison skipped.")
        return
    path, cfg = imu_cfg
    c = cfg.get("imu0") or {}
    rep.info(f"config {os.path.basename(path)}: accel_nd={c.get('accelerometer_noise_density')} "
             f"accel_rw={c.get('accelerometer_random_walk')} gyro_nd={c.get('gyroscope_noise_density')} "
             f"gyro_rw={c.get('gyroscope_random_walk')} update_rate={c.get('update_rate')}")
    for label, cfg_v, meas in (("accel", c.get("accelerometer_noise_density"), a_nd),
                               ("gyro", c.get("gyroscope_noise_density"), g_nd)):
        if cfg_v is None or meas <= 0:
            continue
        ratio = float(cfg_v) / meas
        if ratio < 1.0:
            rep.verdict("FAIL", f"{label} noise density in config is {ratio:.2f}x the measured AT-REST noise. The "
                                "filter is overconfident in the IMU and will chi2-reject vision. Try 10x.")
        elif ratio < 3.0:
            rep.verdict("WARN", f"{label} noise density in config is only {ratio:.1f}x the at-rest noise; motion "
                                "and vibration push the real noise well above that. Try 10x as an experiment.")
        else:
            rep.verdict("PASS", f"{label} noise density in config is {ratio:.1f}x the at-rest noise.")
    if c.get("update_rate") and abs(float(c["update_rate"]) - rate) / rate > 0.1:
        rep.verdict("WARN", f"imu update_rate is {c['update_rate']} Hz but the bag's IMU runs at ~{rate:.0f} Hz.")


def check_quality(rep, d, out):
    rep.section("6. Image quality (sampled frames)")
    if d.get("bad_encoding"):
        rep.verdict("WARN", f"Unhandled image encoding '{d['bad_encoding']}'; quality checks skipped for it.")
    if cv2 is None:
        rep.verdict("WARN", "python3 OpenCV not importable; only brightness checked and no PNGs saved.")
    med_mean = {}
    for side in ("left", "right"):
        qs = d["q_" + side]
        if not qs:
            rep.info(f"{side}: no decodable frames sampled.")
            continue

        def col(key):
            return np.array([q[key] for q in qs if key in q], dtype=float)

        mean = col("mean")
        med_mean[side] = float(np.median(mean))
        rep.info(f"{side}: {len(qs)} samples  brightness median {med_mean[side]:.0f}/255 (min {mean.min():.0f})  "
                 f"saturated {np.median(col('sat')):.1f}%  near-black {np.median(col('dark')):.1f}%")
        if med_mean[side] < 40:
            rep.verdict("WARN", f"{side} images are dark (median {med_mean[side]:.0f}/255): auto-exposure will run "
                                "long and blur on motion. Add light or lock a short exposure with more gain.")
        corners, sharp, dots = col("corners"), col("sharp"), col("dots")
        if len(corners) and len(sharp) and len(dots):
            blurry = int((sharp < 0.3 * np.median(sharp)).sum())
            rep.info(f"       corners median {np.median(corners):.0f} (min {corners.min():.0f})  "
                     f"blurry samples {blurry}/{len(sharp)}  emitter-dot candidates median {np.median(dots):.0f}")
            if np.median(corners) < 150:
                rep.verdict("WARN", f"{side}: median {np.median(corners):.0f} trackable corners per frame, below "
                                    "the 150 points OpenVINS is asked to track. Scene is too plain or too dark.")
            if np.median(dots) > 800:
                rep.verdict("WARN", f"{side}: many tiny bright blobs per frame; the IR projector may be ON. "
                                    "Check the PNGs for the dot pattern.")
    if len(med_mean) == 2 and abs(med_mean["left"] - med_mean["right"]) > 15:
        rep.verdict("WARN", "Left/right brightness differ by more than 15 levels; stereo KLT matching suffers.")
    rep.info(f"sample frames written to {out}: eyeball them for projector dots and motion blur.")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory, e.g. ~/bags/rs_g5_1102_1958")
    ap.add_argument("--config-dir", default="~/vio_config/openvins_d435i")
    ap.add_argument("--out", default=None, help="output dir (default ~/vio_runs/triage_<bag>)")
    ap.add_argument("--left", help="override left image topic")
    ap.add_argument("--right", help="override right image topic")
    ap.add_argument("--imu", help="override united IMU topic")
    ap.add_argument("--baseline", type=float, default=0.050068, help="stereo baseline in m")
    ap.add_argument("--imu-in-cam0", type=float, nargs=3, default=[0.00552, -0.0051, -0.01174],
                    help="IMU position in the infra1 optical frame (from the driver TF), m")
    ap.add_argument("--sample-every", type=int, default=15, help="image-quality sample stride (frames)")
    args = ap.parse_args()

    bag = os.path.expanduser(args.bag.rstrip("/"))
    args.out = os.path.expanduser(args.out or os.path.join("~/vio_runs", "triage_" + os.path.basename(bag)))
    cfg_dir = os.path.expanduser(args.config_dir)

    rep = Report()
    imucam, imu_cfg = find_configs(cfg_dir) if os.path.isdir(cfg_dir) else (None, None)
    cams = check_extrinsics(rep, imucam, args.baseline, args.imu_in_cam0)
    d = read_bag(bag, args, rep)
    if d is not None:
        check_bag_vs_config(rep, d, cams, imu_cfg)
        check_stereo(rep, d)
        check_image_timing(rep, d)
        stream = check_imu_stream(rep, d)
        check_imu_static(rep, d, stream, imu_cfg)
        check_quality(rep, d, args.out)
    rep.summary()

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "report.txt")
    with open(path, "w") as f:
        f.write("\n".join(rep.lines) + "\n")
    print(f"\nreport written to {path}")


if __name__ == "__main__":
    main()
