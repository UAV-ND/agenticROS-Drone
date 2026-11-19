#!/usr/bin/env python3
"""
imu_correct_bag.py - write a COPY of a bag with the D435i accelerometer corrected: a = (a_meas - bias) / scale.

    python3 imu_correct_bag.py ~/bags/rs_g5_1102_1834            # -> ~/bags/rs_g5_1102_1834_imucal
    python3 imu_correct_bag.py <bag> --cal ~/vio_runs/imu_six_face/result.json --out <dir>

Bias/scale come from imu_six_face.py. Every other topic is copied byte-for-byte. The input bag is
only read; the output must not already exist. This emulates what OpenVINS would see after an
on-device IMU calibration (rs-imu-calibration), so a replay A/B isolates the accel-calibration effect.
"""

import argparse
import json
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--cal", default="~/vio_runs/imu_six_face/result.json")
    ap.add_argument("--imu", default="/camera/camera/imu")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    src = os.path.realpath(os.path.expanduser(a.bag))
    out = os.path.expanduser(a.out) if a.out else src.rstrip("/") + "_imucal"
    if os.path.exists(out):
        sys.exit(f"{out} already exists; refusing to overwrite.")
    with open(os.path.expanduser(a.cal)) as f:
        cal = json.load(f)
    bias, scale = np.array(cal["bias"]), np.array(cal["scale"])
    print(f"bias {bias.round(4)}  scale {scale.round(5)}")

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=out, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    for t in r.get_all_topics_and_types():
        w.create_topic(t)

    n_imu = n = 0
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == a.imu:
            m = deserialize_message(data, Imu)
            v = (np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]) - bias) / scale
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(x) for x in v)
            data = serialize_message(m)
            n_imu += 1
        w.write(topic, data, ts)
        n += 1
    del w
    with open(os.path.join(out, "IMUCAL.txt"), "w") as f:
        f.write(f"Copy of {src} with {a.imu} accel = (a - bias)/scale\nbias {bias.tolist()}\nscale {scale.tolist()}\n"
                f"from {a.cal}\n")
    print(f"wrote {out}: {n} messages, {n_imu} IMU corrected")


if __name__ == "__main__":
    main()
