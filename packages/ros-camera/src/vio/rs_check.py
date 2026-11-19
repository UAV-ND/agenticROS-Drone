#!/usr/bin/env python3
"""
rs_check.py - is the live D435i set up the way VIO needs?

Start the camera exactly as you would for VIO, then run this in a second terminal.
Hold the camera still and upright for the first 2 seconds.

    ros2 launch realsense2_camera rs_launch.py config_file:=$HOME/vio_config/d435i_vio.yaml
    python3 rs_check.py                  # options: --node /camera/camera --ns /camera/camera --seconds 8

It reads the node's ACTUAL running parameters (not the config file), checks the USB
link, and listens to the image and IMU topics. Read-only.
"""

import argparse
import glob
import os
import subprocess
import threading
import time

import numpy as np
import yaml

try:
    import cv2
except ImportError:
    cv2 = None

G = 9.806
RESULTS = []


def say(level, msg):
    print(f"  [{level}] {msg}")
    RESULTS.append((level, msg))


def note(msg):
    print(f"         {msg}")


# --------------------------------------------------------------------------
def check_usb():
    print("\n== USB link")
    hits = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                vendor = f.read().strip()
            with open(os.path.join(d, "idProduct")) as f:
                product = f.read().strip()
            if vendor == "8086" and product == "0b3a":
                with open(os.path.join(d, "speed")) as f:
                    hits.append((os.path.basename(d), float(f.read().strip())))
        except (OSError, ValueError):
            continue
    if not hits:
        say("FAIL", "No D435i (USB id 8086:0b3a) found on the bus.")
    for dev, speed in hits:
        if speed >= 5000:
            say("PASS", f"D435i on USB 3 ({speed:.0f} Mb/s) at {dev}.")
        else:
            say("FAIL", f"D435i enumerated at {speed:.0f} Mb/s (USB 2) at {dev}. Use a USB 3 port and cable; "
                        "stereo IR plus IMU at 848x480 needs it.")


# --------------------------------------------------------------------------
def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


def dump_params(node):
    try:
        r = subprocess.run(["ros2", "param", "dump", node], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired) as e:
        say("FAIL", f"'ros2 param dump {node}' failed: {e}")
        return None
    if r.returncode != 0 or not r.stdout.strip():
        say("FAIL", f"'ros2 param dump {node}' failed. Is the camera running and ROS sourced? "
                    f"{r.stderr.strip()[:200]}")
        return None
    doc = yaml.safe_load(r.stdout) or {}
    block = next(iter(doc.values()), {}) if doc else {}
    return flatten(block.get("ros__parameters", block))


def lookup(params, exact, suffix_ok=False):
    if exact in params:
        return exact, params[exact]
    if suffix_ok:
        tail = exact.split(".")[-1]
        for k, v in sorted(params.items()):
            if k.endswith("." + tail) or k == tail:
                return k, v
    return None, None


def check_params(node):
    print(f"\n== Node parameters ({node}, as actually running)")
    p = dump_params(node)
    if p is None:
        return

    k, v = lookup(p, "depth_module.emitter_enabled")
    if v is None:
        say("WARN", "depth_module.emitter_enabled not found; check the projector by eye (and the dot count below).")
    elif int(v) == 0:
        say("PASS", "IR projector off (depth_module.emitter_enabled = 0).")
    else:
        say("FAIL", f"IR projector is ON ({k} = {v}). Set depth_module.emitter_enabled: 0 in d435i_vio.yaml; "
                    "the command-line argument is ignored.")

    k, v = lookup(p, "unite_imu_method")
    if v is None:
        say("WARN", "unite_imu_method not found.")
    elif int(v) == 2:
        say("PASS", "unite_imu_method = 2 (linear_interpolation).")
    elif int(v) == 1:
        say("WARN", "unite_imu_method = 1 (copy): accel is stair-stepped between samples. Set it to 2.")
    else:
        say("FAIL", f"unite_imu_method = {v}: no united /imu topic for OpenVINS. Set it to 2.")

    k, v = lookup(p, "enable_sync")
    if v is True:
        say("PASS", "enable_sync = true (infra1/infra2 published as matched framesets).")
    else:
        say("WARN", f"enable_sync = {v}. Set it true so left and right frames always share a timestamp.")

    for name in ("enable_infra1", "enable_infra2", "enable_gyro", "enable_accel"):
        k, v = lookup(p, name)
        if v is not True:
            say("FAIL", f"{name} = {v}; VIO needs it true.")

    k, v = lookup(p, "accel_fps", suffix_ok=True)
    if v is not None:
        if int(v) in (0, 63):
            say("WARN", f"{k} = {v} (63 Hz default). Set 250 Hz for VIO.")
        else:
            note(f"{k} = {v}")
    k, v = lookup(p, "gyro_fps", suffix_ok=True)
    if v is not None:
        note(f"{k} = {v} (0 means the 200 Hz default)")

    for name in ("depth_module.infra_profile", "depth_module.profile"):
        k, v = lookup(p, name)
        if v is not None:
            note(f"{k} = {v}")
            break

    k, v = lookup(p, "depth_module.enable_auto_exposure")
    if v is True:
        note("depth_module auto-exposure ON. Fine if frames are bright and sharp; if the triage shows dark or "
             "blurry frames, lock a short exposure (enable_auto_exposure false, exposure in us, gain up).")
    elif v is False:
        _, e = lookup(p, "depth_module.exposure")
        _, g = lookup(p, "depth_module.gain")
        note(f"depth_module exposure locked: exposure = {e}, gain = {g}")

    k, v = lookup(p, "enable_depth")
    if v is True:
        note("enable_depth = true. VIO doesn't use depth; turning it off frees USB bandwidth and CPU.")
    k, v = lookup(p, "global_time_enabled", suffix_ok=True)
    if v is not None:
        note(f"{k} = {v}")


# --------------------------------------------------------------------------
def to_gray8(msg):
    if msg.encoding.lower() not in ("mono8", "8uc1"):
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return np.ascontiguousarray(buf.reshape(msg.height, msg.step)[:, :msg.width])


def emitter_dots(img):
    tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    _, bw = cv2.threshold(tophat, 30, 255, cv2.THRESH_BINARY)
    _, _, stats, _ = cv2.connectedComponentsWithStats(bw, connectivity=8)
    return int(np.count_nonzero(stats[1:, cv2.CC_STAT_AREA] <= 12))


def check_topics(ns, seconds):
    print(f"\n== Live topics for {seconds:.0f} s")
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, Imu
    except ImportError as e:
        say("FAIL", f"rclpy not importable ({e}). Source /opt/ros/jazzy/setup.bash first.")
        return

    topics = {"left": f"{ns}/infra1/image_rect_raw", "right": f"{ns}/infra2/image_rect_raw", "imu": f"{ns}/imu"}
    data = {"left": [], "right": [], "imu": [], "acc": [], "gyr": [], "lat": []}
    last = {}
    lock = threading.Lock()

    rclpy.init()
    node = Node("rs_check")
    qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)

    def image_cb(side):
        def cb(msg):
            ns_stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            now = node.get_clock().now().nanoseconds
            with lock:
                data[side].append(ns_stamp)
                if side == "left":
                    data["lat"].append((now - ns_stamp) / 1e6)
                last[side] = msg
        return cb

    def imu_cb(msg):
        with lock:
            data["imu"].append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
            a, w = msg.linear_acceleration, msg.angular_velocity
            data["acc"].append((a.x, a.y, a.z))
            data["gyr"].append((w.x, w.y, w.z))

    node.create_subscription(Image, topics["left"], image_cb("left"), qos)
    node.create_subscription(Image, topics["right"], image_cb("right"), qos)
    node.create_subscription(Imu, topics["imu"], imu_cb, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    print("  listening... keep the camera still and upright for the first 2 s")
    time.sleep(seconds)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()

    with lock:
        L = np.array(data["left"], dtype=np.int64)
        R = np.array(data["right"], dtype=np.int64)
        I = np.array(data["imu"], dtype=np.int64)
        acc = np.array(data["acc"], dtype=float)
        gyr = np.array(data["gyr"], dtype=float)
        lat = np.array(data["lat"], dtype=float)
        last_left = last.get("left")

    for side, arr in (("left", L), ("right", R), ("imu", I)):
        if len(arr) < 5:
            say("FAIL", f"{topics[side]}: {len(arr)} messages. Wrong namespace (--ns) or stream not enabled?")
    if len(L) < 5 or len(I) < 5:
        return

    def rate(arr):
        span = (arr.max() - arr.min()) / 1e9
        return (len(arr) - 1) / span if span > 0 else 0.0

    r_l, r_r, r_i = rate(L), rate(R) if len(R) > 1 else 0.0, rate(I)
    note(f"rates (from stamps): infra1 {r_l:.1f} Hz, infra2 {r_r:.1f} Hz, imu {r_i:.1f} Hz")
    if r_l < 27:
        say("WARN", f"infra1 at {r_l:.1f} Hz, below the expected 30 Hz.")

    dt = np.diff(np.sort(L)) / 1e6
    dt = dt[dt > 0]
    if len(dt) > 3:
        med = float(np.median(dt))
        jitter = float(dt[(dt > 0.5 * med) & (dt < 1.5 * med)].std())
        note(f"infra1 frame period {med:.3f} ms, jitter {jitter:.3f} ms")
        if jitter > 1.0:
            say("WARN", f"{jitter:.2f} ms frame-stamp jitter: stamps look like host arrival time, not the camera clock.")

    if len(R) > 5:
        Rs = np.sort(R)
        idx = np.clip(np.searchsorted(Rs, L), 1, len(Rs) - 1)
        near = np.minimum(np.abs(Rs[idx] - L), np.abs(Rs[idx - 1] - L)) / 1e6
        orphans = int((near > 1.0).sum())
        if orphans == 0:
            say("PASS", "Every infra1 frame has an infra2 frame at the same stamp.")
        else:
            say("WARN", f"{orphans}/{len(L)} infra1 frames have no infra2 frame within 1 ms.")

    if len(lat):
        med_lat = float(np.median(lat))
        note(f"camera-to-ROS latency (now - stamp): median {med_lat:.1f} ms, p95 {np.percentile(lat, 95):.1f} ms")
        if abs(med_lat) > 1000:
            say("WARN", f"Image stamps are {med_lat / 1000:.1f} s off the ROS clock. Fine for bag replays, but "
                        "MAVROS and EKF3 later need stamps on the system clock (check global_time_enabled).")

    step = np.diff(I)
    if (step <= 0).any():
        say("FAIL", f"IMU stamps not strictly increasing ({int((step < 0).sum())} backwards, "
                    f"{int((step == 0).sum())} duplicates).")
    else:
        say("PASS", "IMU stamps strictly increasing.")
    if len(acc) > 2:
        rep = float(np.all(np.diff(acc, axis=0) == 0, axis=1).mean())
        if rep > 0.25:
            say("WARN", f"{rep * 100:.0f}% of IMU messages repeat the previous accel value (copy-style union).")

    first = I < I.min() + 2_000_000_000
    if first.sum() > 50:
        gmean = float(np.linalg.norm(gyr[first], axis=1).mean())
        amag = float(np.linalg.norm(acc[first].mean(axis=0)))
        if gmean > 0.03:
            note(f"camera moved during the first 2 s (mean |gyro| {gmean:.3f} rad/s); accel magnitude check skipped")
        else:
            err = abs(amag - G)
            level = "PASS" if err <= 0.08 else ("WARN" if err <= 0.25 else "FAIL")
            say(level, f"|accel| at rest = {amag:.3f} m/s^2 ({err:.2f} from g). "
                       + ("" if level == "PASS" else "Run rs-imu-calibration to write an IMU calibration to the camera."))

    if last_left is not None:
        img = to_gray8(last_left)
        if img is not None:
            mean = float(img.mean())
            line = f"last infra1 frame: brightness {mean:.0f}/255"
            if cv2 is not None:
                pts = cv2.goodFeaturesToTrack(img, maxCorners=1000, qualityLevel=0.01, minDistance=8)
                dots = emitter_dots(img)
                line += f", corners {0 if pts is None else len(pts)}, emitter-dot candidates {dots}"
                if dots > 800:
                    say("WARN", f"{dots} tiny bright blobs in the frame: the projector may really be on. Look at "
                                "rqt_image_view for the dot pattern.")
            note(line)
            if mean < 40:
                say("WARN", "Frame is dark. Add light before recording VIO bags.")


# --------------------------------------------------------------------------
def check_imu_calibration_tool():
    print("\n== IMU calibration tool")
    try:
        r = subprocess.run(["python3", "-c", "import pyrealsense2"], capture_output=True, timeout=30)
        ok = r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        ok = False
    if ok:
        note("pyrealsense2 imports, so librealsense's tools/rs-imu-calibration/rs-imu-calibration.py can run "
             "(stop the ROS camera first).")
    else:
        note("pyrealsense2 is not importable. rs-imu-calibration needs it: rebuild librealsense with "
             "-DBUILD_PYTHON_BINDINGS=ON if the triage flags the IMU.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", default="/camera/camera", help="realsense node name")
    ap.add_argument("--ns", default="/camera/camera", help="topic namespace")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    check_usb()
    check_params(args.node)
    check_topics(args.ns.rstrip("/"), args.seconds)
    check_imu_calibration_tool()

    print("\n== SUMMARY (worst first)")
    order = {"FAIL": 0, "WARN": 1, "PASS": 2}
    for level, msg in sorted(RESULTS, key=lambda r: order[r[0]]):
        print(f"  [{level}] {msg}")


if __name__ == "__main__":
    main()
