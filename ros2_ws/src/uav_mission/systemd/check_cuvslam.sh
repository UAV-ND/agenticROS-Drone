#!/bin/bash
# Check that cuVSLAM is up and tracking (read-only).   bash check_cuvslam.sh
# cuVSLAM needs about a minute after (re)start before it publishes.
# PASS needs: container running, /visual_slam/tracking/odometry >= 20 Hz with small pose variance.
# Normal is 26-30 Hz: cuVSLAM skips a camera frame now and then when the Orin is busy.
source "$(dirname "$(readlink -f "$0")")/uav_env.sh" >/dev/null
fail=0
ok()  { echo "  PASS  $*"; }
bad() { echo "  FAIL  $*"; fail=1; }

if [ -f /etc/systemd/system/uav-cuvslam.service ]; then
    echo "uav-cuvslam.service: $(systemctl is-active uav-cuvslam.service), boot start $(systemctl is-enabled uav-cuvslam.service)"
else
    echo "uav-cuvslam.service: not installed (cuVSLAM started by hand?)"
fi
st=$(docker ps --filter name=^uav_cuvslam$ --format '{{.Status}}')
[ -n "$st" ] && ok "container uav_cuvslam: $st" || bad "container uav_cuvslam not running"
pgrep -f realsense2_camera_node >/dev/null && ok "camera node running" || bad "camera node not running (uav-camera.service)"

python3 - <<'PY' || fail=1
import sys, time, math, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('check_cuvslam'); msgs = []
n.create_subscription(Odometry, '/visual_slam/tracking/odometry', msgs.append,
                         QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.time() + 2
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)   # discovery
msgs.clear(); end = time.time() + 5
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
hz = len(msgs) / 5
bad = hz < 20
print(f"  {'PASS' if hz >= 20 else 'FAIL'}  VIO odometry {hz:5.1f} Hz (need >= 20, normal 26-30)")
if msgs:
    var = max(msgs[-1].pose.covariance[0], msgs[-1].pose.covariance[7], msgs[-1].pose.covariance[14])
    good = var <= 0.02
    print(f"  {'PASS' if good else 'FAIL'}  pose variance {var:.2g} m^2 (need <= 0.02, the bridge's limit)")
    p = msgs[-1].pose.pose.position
    print(f"        position x {p.x:.2f} y {p.y:.2f} z {p.z:.2f} m (from where cuVSLAM started)")
    bad |= not good
sys.exit(1 if bad else 0)
PY
[ $fail = 0 ] && echo "RESULT: VIO READY" || echo "RESULT: VIO NOT READY"
exit $fail
