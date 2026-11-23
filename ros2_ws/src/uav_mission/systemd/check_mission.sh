#!/bin/bash
# Check the mission service (read-only).   bash check_mission.sh
# PASS = the mission node is up, IDLE, and would accept a start right now (no preflight problems),
# and the RC trigger sees ch6 LOW (ready for LOW -> HIGH).
source "$(dirname "$(readlink -f "$0")")/uav_env.sh" >/dev/null
if [ -f /etc/systemd/system/uav-mission.service ]; then
    echo "uav-mission.service: $(systemctl is-active uav-mission.service), boot start $(systemctl is-enabled uav-mission.service)"
else
    echo "uav-mission.service: not installed"
fi
echo "parameters: $UAV_MISSION_PARAMS"
python3 - <<'PY'
import json, sys, time, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('check_mission')
last = {}
n.create_subscription(String, '/uav/mission/state', lambda m: last.__setitem__('m', m.data), 10)
n.create_subscription(String, '/uav/rc_trigger/status', lambda m: last.__setitem__('rc', m.data), 10)
t0 = time.time()
while time.time() - t0 < 4 and not ('m' in last and 'rc' in last):
    rclpy.spin_once(n, timeout_sec=0.2)
fail = 0
def check(ok, msg):
    global fail
    print(('PASS  ' if ok else 'FAIL  ') + msg); fail |= not ok
m = json.loads(last['m']) if 'm' in last else None
check(m is not None, 'mission node publishing /uav/mission/state' if m else 'no /uav/mission/state (uav-mission not running?)')
if m:
    st = m.get('state')
    done = st in ('DONE', 'PILOT', 'ABORTED') and not m.get('armed')   # finished on the ground: next SD flip restarts it
    check(st == 'IDLE' or done, f"mission state {st} ({m.get('reason') or 'waiting for start'})"
          + (' - finished, a new SD flip starts it again' if done else ''))
    probs = m.get('preflight_problems') or []
    check(not probs, 'preflight: would accept a start now' if not probs else 'preflight would REFUSE: ' + '; '.join(probs))
    print(f"        mode {m.get('mode')}, armed {m.get('armed')}, EKF source set {m.get('source_set')}, "
          f"vio_ok {m.get('vio_ok')}, home {m.get('home_ok')}, battery {m.get('battery')}")
    if m.get('fc_errors'):
        print(f"        recent Cube warnings: {m['fc_errors'][-3:]}")
rc = last.get('rc')
try:
    rc = json.loads(rc).get('status', rc)
except Exception:
    pass
# after a run that ended with SD flipped up, the status still reads "switch LOW in flight: abort": SD is LOW = ready
rc_ok = rc is not None and ('ready' in str(rc) or 'switch LOW' in str(rc))
check(rc_ok, f"RC trigger: {rc}" + (' (SD is LOW: ready)' if rc_ok and 'ready' not in str(rc) else '') if rc else 'no /uav/rc_trigger/status')
print('RESULT: MISSION READY (flip ch6 LOW -> HIGH to start)' if not fail else 'RESULT: MISSION NOT READY')
sys.exit(1 if fail else 0)
PY
