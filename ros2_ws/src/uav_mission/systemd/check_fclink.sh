#!/bin/bash
# Check the Cube link + VIO bridge (read-only).   bash check_fclink.sh
# PASS needs: MAVROS connected, Cube streaming attitude/position, RC receiver seen,
# VIO bridge TRACKING and sending odometry to the Cube. Also prints mode, arming, switches.
source "$(dirname "$(readlink -f "$0")")/uav_env.sh" >/dev/null
if [ -f /etc/systemd/system/uav-fclink.service ]; then
    echo "uav-fclink.service: $(systemctl is-active uav-fclink.service), boot start $(systemctl is-enabled uav-fclink.service)"
else
    echo "uav-fclink.service: not installed (fc_link started by hand?)"
fi
python3 - <<'PY'
import json, sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, RCIn
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
Q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
rclpy.init(); n = rclpy.create_node('check_fclink')
c = {'imu': 0, 'pose': 0, 'odom': 0, 'rc': 0}; last = {}
def cnt(k):
    def f(m): c[k] += 1; last[k] = m
    return f
n.create_subscription(Imu, '/mavros/imu/data', cnt('imu'), Q)
n.create_subscription(PoseStamped, '/mavros/local_position/pose', cnt('pose'), Q)
n.create_subscription(Odometry, '/mavros/odometry/out', cnt('odom'), Q)
n.create_subscription(RCIn, '/mavros/rc/in', cnt('rc'), Q)
n.create_subscription(State, '/mavros/state', lambda m: last.__setitem__('state', m), Q)
n.create_subscription(String, '/uav/fc_status', lambda m: last.__setitem__('fc', m), 10)
end = time.time() + 2
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
for k in c: c[k] = 0
T = 5; end = time.time() + T
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.01)
fail = False
def check(ok, msg):
    global fail
    print(f"  {'PASS' if ok else 'FAIL'}  {msg}"); fail |= not ok
st = last.get('state')
check(st is not None and st.connected, 'MAVROS connected to the Cube' if st else 'no /mavros/state (MAVROS not running?)')
check(c['imu'] / T >= 5, f"Cube attitude {c['imu']/T:.1f} Hz (need >= 5; 0 = SR0_* stream rates wrong)")
check(c['pose'] / T >= 5, f"Cube local position {c['pose']/T:.1f} Hz (need >= 5)")
rc = last.get('rc'); ch = list(rc.channels) if rc else []
check(len(ch) >= 10 and ch[0] > 0, 'RC receiver seen' if len(ch) >= 10 else 'no RC channels (transmitter off?)')
fc = {}
try: fc = json.loads(last['fc'].data)
except Exception: pass
check(fc.get('bridge_state') == 'TRACKING', f"VIO bridge state {fc.get('bridge_state')} (need TRACKING)")
check(c['odom'] / T >= 20, f"VIO to Cube {c['odom']/T:.1f} Hz (need >= 20)")
if st: print(f"        mode {st.mode}, {'ARMED' if st.armed else 'disarmed'}")
if fc: print(f"        EKF source set {fc.get('source_set')}, Cube using VIO: {fc.get('ext_nav_in_use')}, warnings: {fc.get('warnings')}")
if len(ch) >= 10:
    pos = lambda v: 'LOW' if v < 1300 else 'HIGH' if v > 1700 else 'MID'
    print(f"        ch5 arm {pos(ch[4])} ({ch[4]}), ch6 mission {pos(ch[5])} ({ch[5]}), ch10 source {pos(ch[9])} ({ch[9]})")
print('RESULT: CUBE LINK READY' if not fail else 'RESULT: CUBE LINK NOT READY')
sys.exit(1 if fail else 0)
PY
