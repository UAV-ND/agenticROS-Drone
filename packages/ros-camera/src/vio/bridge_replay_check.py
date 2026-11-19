#!/usr/bin/env python3
"""Offline check of the VIO bridge gating on recorded cuVSLAM runs (no live ROS graph needed).

  source /opt/ros/jazzy/setup.bash
  python3 bridge_replay_check.py ~/vio_runs/<run>/odom [more runs...] [--pitch 30] [--csv out.csv]

Reads /visual_slam/tracking/odometry from each run's odometry bag, feeds it through
vio_bridge_core.Bridge exactly as the live node does (minus the clock-age check), and prints
what the bridge would have published: share of frames published, every rejection, and the
start-to-end distance of the raw vs bridged track (for loop bags that return to the start,
that distance is the drift).
"""
import argparse
import os
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vio_bridge_core import Bridge, GateParams, Sample, mount_transform, se3  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

TOPIC = '/visual_slam/tracking/odometry'


def read_samples(bag_dir):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != TOPIC:
            continue
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        T = se3(Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [p.x, p.y, p.z])
        tw = m.twist.twist
        yield Sample(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, T,
                     np.array([tw.linear.x, tw.linear.y, tw.linear.z]),
                     np.array([tw.angular.x, tw.angular.y, tw.angular.z]),
                     np.array(m.pose.covariance).reshape(6, 6),
                     np.array(m.twist.covariance).reshape(6, 6))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--pitch', type=float, default=0.0, help='mount pitch-down deg (0 = handheld/cart rig)')
    ap.add_argument('--csv', help='write the bridged track (t,x,y,z,published) for the first bag')
    args = ap.parse_args()
    for i, bag in enumerate(args.bags):
        bag = os.path.expanduser(bag)
        b = Bridge(mount_transform(0, 0, 0, 0, args.pitch, 0), GateParams(max_age_s=0))
        raw, out, rows, t0 = [], [], [], None
        for s in read_samples(bag):
            t0 = s.stamp if t0 is None else t0
            o = b.process(s)
            if b.prev_raw is not None:
                raw.append(b.prev_raw[:3, 3].copy())
                if b.prev_corr is not None:
                    rows.append((s.stamp - t0, *b.prev_corr[:3, 3], o is not None))
            if o is not None:
                out.append(o.p)
        st = b.stats
        name = os.path.basename(os.path.dirname(bag.rstrip('/')))
        print(f'== {name}')
        if not out:
            print('   nothing published')
            continue
        raw, out = np.array(raw), np.array(out)
        path = np.sum(np.linalg.norm(np.diff(out, axis=0), axis=1))
        print(f'   frames {st.received}, published {st.published} ({100*st.published/st.received:.1f}%), '
              f'jumps {st.jumps} (gaps {st.gaps}), high-cov frames {st.high_cov}, holds {st.holds}, '
              f'stale {st.stale}')
        print(f'   start->end: raw {np.linalg.norm(raw[-1]-raw[0]):.3f} m | bridged {np.linalg.norm(out[-1]-out[0]):.3f} m'
              f' | bridged path {path:.1f} m | jump displacement absorbed {st.jump_absorbed_m:.2f} m')
        steps = np.linalg.norm(np.diff(out, axis=0), axis=1)
        print(f'   largest step between published poses: {steps.max():.3f} m')
        shown = {}
        for stamp, kind, detail in st.events:
            k = (kind, round(stamp - t0))
            if k in shown:
                continue
            shown[k] = 1
            if len(shown) <= 25:
                print(f'     t={stamp - t0:7.2f}  {kind:8s} {detail}')
        if len(shown) > 25:
            print(f'     ... {len(shown) - 25} more (grouped per second)')
        if args.csv and i == 0:
            np.savetxt(args.csv, np.array(rows), delimiter=',', header='t,x,y,z,published', comments='')


if __name__ == '__main__':
    main()
