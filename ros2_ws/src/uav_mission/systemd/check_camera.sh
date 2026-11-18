#!/bin/bash
# Check that the D435i is up and streaming correctly for VIO (read-only).   bash check_camera.sh
# PASS needs: USB 3, IR left/right camera ~30 fps with few dropped frames, IMU >= 150 Hz, IR emitter off.
source "$(dirname "$(readlink -f "$0")")/uav_env.sh" >/dev/null
fail=0
ok()  { echo "  PASS  $*"; }
bad() { echo "  FAIL  $*"; fail=1; }

if [ -f /etc/systemd/system/uav-camera.service ]; then
    echo "uav-camera.service: $(systemctl is-active uav-camera.service), boot start $(systemctl is-enabled uav-camera.service)"
else
    echo "uav-camera.service: not installed (camera started by hand?)"
fi
pgrep -f realsense2_camera_node >/dev/null && ok "camera node running" || bad "camera node not running"

speed=""
for d in /sys/bus/usb/devices/*; do
    [ "$(cat $d/idVendor 2>/dev/null)" = 8086 ] && [ "$(cat $d/idProduct 2>/dev/null)" = 0b3a ] && speed=$(cat $d/speed)
done
if [ -z "$speed" ]; then bad "D435i not on USB"
elif [ "$speed" -ge 5000 ]; then ok "USB 3 (${speed} Mb/s)"
else bad "USB 2 (${speed} Mb/s): re-seat the cable in a USB 3 port, then restart the camera"; fi

python3 - <<'PY' || fail=1
# Frame rate is measured from the small per-frame metadata topics (frame timestamps), not by
# counting full images: a Python subscriber can't keep up with 2x30 Hz images while cuVSLAM also
# reads them, which made healthy cameras look slow. Images are only checked to be flowing.
import sys, time, rclpy
import numpy as np
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from realsense2_camera_msgs.msg import Metadata
Q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)   # deep queue: don't lose stamps
rclpy.init(); n = rclpy.create_node('check_camera')
st = {1: [], 2: []}; img = {1: 0, 2: 0}; imu = [0]
for i in (1, 2):
    n.create_subscription(Metadata, f'/camera/camera/infra{i}/metadata',
                          lambda m, i=i: st[i].append(round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 4)), Q)
    n.create_subscription(Image, f'/camera/camera/infra{i}/image_rect_raw',
                          lambda m, i=i: img.__setitem__(i, img[i] + 1), qos_profile_sensor_data, raw=True)
n.create_subscription(Imu, '/camera/camera/imu', lambda m: imu.__setitem__(0, imu[0] + 1), qos_profile_sensor_data)
end = time.time() + 2
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)   # discovery
for i in (1, 2): st[i].clear(); img[i] = 0
imu[0] = 0; T = 5; end = time.time() + T
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.02)
bad = False
allst = np.unique(st[1] + st[2])          # a frame missing on one side only = checker loss, not camera
for i, name in ((1, 'IR left '), (2, 'IR right')):
    s = np.unique(st[i])
    if len(s) < 10:
        print(f"  FAIL  {name} no frames"); bad = True; continue
    d = np.diff(s); fps = 1.0 / np.median(d)
    da = np.diff(allst); drops = int(np.sum(np.round(da * fps) - 1))     # frames missing on BOTH sides
    good = fps >= 27 and drops <= 0.05 * len(s) and img[i] / T >= 10
    print(f"  {'PASS' if good else 'FAIL'}  {name} camera {fps:4.1f} fps, {drops} dropped frames in {T} s, images arriving")
    bad |= not good
hz = imu[0] / T
print(f"  {'PASS' if hz >= 150 else 'FAIL'}  IMU      {hz:6.1f} Hz (need >= 150)")
bad |= hz < 150
sys.exit(1 if bad else 0)
PY

em=$(timeout 10 ros2 param get /camera/camera depth_module.emitter_enabled 2>/dev/null | grep -o '[0-9]\+$')
if [ "$em" = 0 ]; then ok "IR emitter off"
elif [ -z "$em" ]; then bad "could not read the emitter setting"
else bad "IR emitter is ON ($em): the dot pattern breaks VIO; check d435i_vio.yaml"; fi

[ $fail = 0 ] && echo "RESULT: CAMERA READY" || echo "RESULT: CAMERA NOT READY"
exit $fail
