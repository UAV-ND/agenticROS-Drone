#!/bin/bash
# Check that the D435i is up and streaming correctly for VIO (read-only).   bash check_camera.sh
# PASS needs: USB 3, IR left/right ~30 Hz, IMU >= 150 Hz, IR emitter off.
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
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
topics = {'/camera/camera/infra1/image_rect_raw': (Image, 27, 'IR left'),
          '/camera/camera/infra2/image_rect_raw': (Image, 27, 'IR right'),
          '/camera/camera/imu': (Imu, 150, 'IMU')}
rclpy.init(); n = rclpy.create_node('check_camera'); c = {t: 0 for t in topics}
for t, (cls, _, _) in topics.items():
    n.create_subscription(cls, t, lambda m, t=t: c.__setitem__(t, c[t] + 1), qos_profile_sensor_data)
end = time.time() + 2
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)   # discovery
c = {t: 0 for t in topics}; end = time.time() + 5
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
bad = False
for t, (_, need, name) in topics.items():
    hz = c[t] / 5
    print(f"  {'PASS' if hz >= need else 'FAIL'}  {name:8s} {hz:6.1f} Hz (need >= {need})")
    bad |= hz < need
sys.exit(1 if bad else 0)
PY

em=$(timeout 10 ros2 param get /camera/camera depth_module.emitter_enabled 2>/dev/null | grep -o '[0-9]\+$')
if [ "$em" = 0 ]; then ok "IR emitter off"
elif [ -z "$em" ]; then bad "could not read the emitter setting"
else bad "IR emitter is ON ($em): the dot pattern breaks VIO; check d435i_vio.yaml"; fi

[ $fail = 0 ] && echo "RESULT: CAMERA READY" || echo "RESULT: CAMERA NOT READY"
exit $fail
