#!/usr/bin/env python3
"""
imu_six_face.py - D435i accelerometer bias/scale from a 6-face static test, through ROS (no pyrealsense2).

Camera must be running (d435i_vio.yaml). Rest the camera on each face, perfectly still, then:

    python3 imu_six_face.py capture <label>    # e.g. "upright", "upside-down", "lens-up"
    python3 imu_six_face.py solve              # after all 6

The label is only kept for the record: the script detects which axis carries gravity and its sign,
and rejects a capture that is not still or not within 15 deg of an axis.

Model per axis k:  a_meas_k = s_k * a_true_k + b_k.  With +g and -g on axis k:
    s_k = (m_plus - m_minus) / (2 g),   b_k = (m_plus + m_minus) / 2
Output: per-axis bias and scale, plus a Ta matrix for OpenVINS. OpenVINS applies Da = inv(Ta) as
a = Da (a_meas - b_a), so Ta = diag(s). Note: OpenVINS static init uses the RAW accel mean, so a large
bias still tilts its gravity estimate; correct the data itself (imu_correct_bag.py) or on-device.
Captures are stored in ~/vio_runs/imu_six_face/captures.json. Read-only with respect to the camera.
"""

import json
import os
import sys
import time

import numpy as np

G = 9.80665  # standard gravity; Notre Dame local g ~9.803, difference is negligible here
STORE = os.path.expanduser("~/vio_runs/imu_six_face/captures.json")
TOPIC = "/camera/camera/imu"


def load():
    if os.path.exists(STORE):
        with open(STORE) as f:
            return json.load(f)
    return []


def capture(label, seconds=4.0):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node("imu_six_face")
    A, W = [], []

    def cb(m):
        A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        W.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    node.create_subscription(Imu, TOPIC, cb, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    A, W = np.array(A), np.array(W)
    if len(A) < 100:
        sys.exit(f"Only {len(A)} IMU samples on {TOPIC}; is the camera running?")
    gyro = np.linalg.norm(W - W.mean(0), axis=1).mean()
    # Robust: median value, stillness from the 95th-percentile deviation (a few spikes don't reject it).
    mean = np.median(A, axis=0)
    dev = np.abs(A - mean).max(axis=1)
    astd = float(np.percentile(dev, 95)) / 2.0
    spikes = int((dev > 0.3).sum())
    if spikes:
        idx = np.where(dev > 0.3)[0]
        print(f"note: {spikes} accel samples >0.3 m/s^2 off the median, at sample "
              f"{idx.min()}..{idx.max()} of {len(A)} (raw std {A.std(0).max():.3f})")
    k = int(np.argmax(np.abs(mean)))
    sign = "+" if mean[k] > 0 else "-"
    tilt = np.degrees(np.arccos(min(1.0, abs(mean[k]) / np.linalg.norm(mean))))
    print(f"{len(A)} samples  mean accel {np.round(mean, 4)}  |a| {np.linalg.norm(mean):.4f}  "
          f"accel std {astd:.3f}  gyro noise {gyro:.4f} rad/s")
    if astd > 0.08 or gyro > 0.02:
        sys.exit("NOT STILL: capture rejected. Let the camera settle on the table and retry.")
    if tilt > 15:
        sys.exit(f"Gravity is {tilt:.1f} deg off the nearest axis: rest the camera flat on a face and retry.")
    key = f"{sign}{'xyz'[k]}"
    caps = [c for c in load() if c["axis"] != key]
    caps.append({"axis": key, "label": label, "mean": mean.tolist(), "tilt_deg": tilt,
                 "n": len(A), "time": time.strftime("%Y-%m-%d %H:%M:%S")})
    os.makedirs(os.path.dirname(STORE), exist_ok=True)
    with open(STORE, "w") as f:
        json.dump(caps, f, indent=1)
    have = sorted(c["axis"] for c in caps)
    need = sorted(set(["+x", "-x", "+y", "-y", "+z", "-z"]) - set(have))
    print(f"Stored axis {key} (tilt {tilt:.1f} deg).  Have {have}.  Still need {need or 'nothing: run solve'}")


def solve():
    caps = {c["axis"]: np.array(c["mean"]) for c in load()}
    missing = [a for a in ["+x", "-x", "+y", "-y", "+z", "-z"] if a not in caps]
    if missing:
        sys.exit(f"Missing faces: {missing}")
    bias, scale = np.zeros(3), np.zeros(3)
    for k, ax in enumerate("xyz"):
        mp, mm = caps["+" + ax][k], caps["-" + ax][k]
        scale[k] = (mp - mm) / (2 * G)
        bias[k] = (mp + mm) / 2
    print("axis   bias (m/s^2)   scale     scale error")
    for k, ax in enumerate("xyz"):
        print(f"  {ax}    {bias[k]:+.4f}       {scale[k]:.5f}   {100 * (scale[k] - 1):+.2f}%")
    # Residual check: corrected |a| on every face should be ~G.
    print("corrected |a| per face:")
    for key, m in sorted(caps.items()):
        c = (m - bias) / scale
        print(f"  {key}: raw {np.linalg.norm(m):.4f} -> corrected {np.linalg.norm(c):.4f}")
    worst_tilt_err = np.degrees(np.arctan(np.abs(bias).max() / G))
    print(f"worst-case gravity direction error from bias alone: {worst_tilt_err:.2f} deg")
    Ta = np.diag(scale)
    print("\nOpenVINS kalibr_imu_chain.yaml  (OpenVINS uses a = inv(Ta) (a_meas - b_a), so Ta = diag(scale)):")
    print("  Ta:")
    for row in Ta:
        print(f"    - [ {row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f} ]")
    out = os.path.join(os.path.dirname(STORE), "result.json")
    with open(out, "w") as f:
        json.dump({"bias": bias.tolist(), "scale": scale.tolist(), "Ta": Ta.tolist()}, f, indent=1)
    print(f"\nSaved {out}")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "solve":
        solve()
    elif len(sys.argv) >= 2 and sys.argv[1] == "capture":
        capture(sys.argv[2] if len(sys.argv) > 2 else "")
    else:
        sys.exit(__doc__)
