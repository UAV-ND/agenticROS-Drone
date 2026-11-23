# 06 Simulation (SITL)

[← wiki home](README.md)

The real mission stack (MAVROS + `mission_node` + ch6 trigger) flies against **ArduCopter 4.6.3 SITL** built
on the Orin (`~/ardupilot`, venv `~/mav_venv`). GPS-denied: SITL's "vicon" plays the role of VIO as external
nav on SERIAL5. Everything runs on **`ROS_DOMAIN_ID=42`**, so it can't reach the real Cube or the running
`uav-mission` service. **Step-by-step commands: `ros2_ws/src/uav_mission/sitl/README.md`.**

Short version:
1. Copy the mission file with `require_vio: false` (no camera in SITL; external nav is on set 1 there).
2. Start SITL with `--serial5=sim:vicon:` and `--add-param-file=sitl/sitl_gps_denied.parm`.
3. `python3 sitl/rc.py` (fake transmitter over UDP 5501), then `ros2 launch uav_mission mission.launch.py
   fcu_url:=tcp://127.0.0.1:5760 rc_trigger:=true rc_channel:=6 params:=<your copy>` on domain 42.
4. Wait for `"preflight_problems": []`, then `rc.py 6=2000` (start), `rc.py 3=1500` once it climbs
   (throttle centred, like the pilot), `rc.py 6=1000` (abort), `rc.py 9=1295` (takeover in ALT_HOLD).

What to check in a run: state sequence `PREFLIGHT → TAKEOFF → PATH → HOVER → LAND → DONE`; distance to each
corner; overshoot past the square; height; peak speed; landing position; that it disarmed by itself.

## Gotchas

- SITL needs the `SR0_*` stream rates (in `sitl_gps_denied.parm`) or MAVROS gets no RC ("no RC data").
- Battery drains ~15–20% per SITL flight; restart SITL (`-w` wipes params) to reset.
- Stop SITL processes **by PID**. `pkill -f <pattern>` can match your own shell.
- `ros2 bag record` killed abruptly can leave an empty bag; a small CSV recorder subscribing to
  `/mavros/local_position/pose`, `/velocity_local`, `/mavros/setpoint_raw/local` and `/uav/mission/state` is more robust.
- SITL has no camera, so VIO quality (especially at speed) can only be tested on the drone.

## Results so far

| Date | Mission | Result |
|---|---|---|
| 2026-11-18 | 3 m square | corners within 7 cm, 4 cm from outline, landed on start; ch6 abort → LAND and mode takeover → PILOT worked |
| 2026-11-22 | 10 m fast square, 5 m/s limit | peak 3.8 m/s, corners 2–5 cm, ≤ 0.4 m past the square (16 m sides: 4.7 m/s) |
| 2026-11-22 | 10 m fast square, Cube default speed (`path_speed 0.0`) | no speed command sent; peak 3.78 m/s, square 18.9 s, corners ≤ 7 cm, ≤ 0.39 m off the outline, height 3.00–3.02 m, landed 1 cm from start |
