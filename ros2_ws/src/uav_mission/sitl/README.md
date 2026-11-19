# SITL test of the autonomous square (no drone needed)

Flies the real mission stack (MAVROS + mission node + ch6 RC trigger) against ArduCopter 4.6.3 SITL
on the Orin, GPS-denied, with SITL's "vicon" standing in for VIO as external nav. Everything runs on
**ROS domain 42**, so it can't reach the real Cube or the running `uav-mission` service.
Last run 2026-11-18: square within 7 cm of every corner, 4 cm of the outline, landed on the start;
ch6 abort -> LAND, mode-switch takeover -> PILOT both worked.

The difference from the drone: `require_vio: false` (no camera in SITL), and the external nav is on
source set 1 instead of 2.

```bash
H=~/Documents/uav-nd/agenticROS-Drone/ros2_ws/src/uav_mission/sitl
sed 's/require_vio: true .*/require_vio: false/' $H/../config/square_3m_vio.yaml > /tmp/square_sitl.yaml

# terminal 1: SITL (no GPS, vicon on SERIAL5, real-drone modes/switches/stream rates)
cd ~/ardupilot/ArduCopter && source ~/mav_venv/bin/activate
../Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --no-mavproxy -w \
    -A "--serial5=sim:vicon:" --add-param-file=$H/sitl_gps_denied.parm

# terminal 2: initial transmitter state (throttle low, SF/ch5 LOW, SD/ch6 LOW, mode LOITER, SG/ch10 LOW)
python3 $H/rc.py
# then the mission stack on domain 42
export ROS_DOMAIN_ID=42; source ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/install/setup.bash
ros2 launch uav_mission mission.launch.py fcu_url:=tcp://127.0.0.1:5760 rc_trigger:=true \
    rc_channel:=6 params:=/tmp/square_sitl.yaml

# terminal 3 (ROS_DOMAIN_ID=42): optional recorder -> pose.csv / state.log in the given folder
python3 $H/rec.py /tmp
```
Wait until `ros2 topic echo /uav/mission/state` shows `"preflight_problems": []` (about 30 s), then:
```bash
python3 $H/rc.py 6=2000      # SD down: start
python3 $H/rc.py 3=1500      # once it climbs: throttle centred (as the pilot does)
python3 $H/rc.py 6=1000      # SD up in flight: abort (lands where it is)
python3 $H/rc.py 9=1295      # mode switch to ALT_HOLD: pilot takeover
```
`rc.py` keeps the last values in `rc_state.json` next to it (delete it to reset).
