# 09 Cube parameters

[← wiki home](README.md)

**Humans set Cube parameters in Mission Planner. Code on the Orin never writes them** (team decision 2026-11-07).
Agents may *read* and *check* parameters, and propose changes for a human to make.

## Files (`packages/ros-camera/src/vio/`)

| File | What |
|---|---|
| `cube_gps_denied.parm` | **The GPS-denied overlay**: VIO/EKF source sets, stream rates, RTL_ALT etc. No calibration or flight modes. Load in Mission Planner in **two passes** (VISO_* appear only after VISO_TYPE=1 and a reboot) |
| `cube_known_good_2026-11-09.parm` | Full backup of a known-good configuration |
| `check_cube_params.py` | **Read-only** checker: compares the live Cube (or a log's PARM dump) against `cube_gps_denied.parm`. Needs the USB port free: `sudo systemctl stop uav-mission` first, start it again after |
| `uavnd_gps_denied_stage1.parm` | Older ARK-Flow-only stage-1 file |

```bash
sudo systemctl stop uav-mission
~/mav_venv/bin/python ~/Documents/uav-nd/agenticROS-Drone/packages/ros-camera/src/vio/check_cube_params.py
sudo systemctl start uav-mission
```
Run it **every flight day**. Parameters have silently reverted twice (2026-11-09 and 2026-11-17: VISO_TYPE 0,
EK3_SRC1_VELXY 0, EK3_SRC2_* 0, RTL_ALT 1500, SR0_* 0), most likely from an older file loaded in Mission
Planner. When that happens the EKF ignores VIO while the Orin side looks healthy.

## Key values

| Parameter | Value | Why |
|---|---|---|
| EK3_SRC1_POSXY / VELXY / POSZ / YAW | 0 / 5 (optical flow) / 1 (baro) / 1 (compass) | set 1 = ARK Flow, the primary |
| EK3_SRC2_POSXY / VELXY / POSZ / YAW | 6 / 6 (ExternalNav) / 1 (baro) / 6 | set 2 = VIO |
| EK3_SRC3_* | no horizontal source | set 3 = SG HIGH: never use |
| VISO_TYPE | 1 (MAVLink) | accept ODOMETRY from MAVROS |
| RC10_OPTION | 90 (EKF source set) | SG switch |
| RC5_OPTION | 153 (arm/disarm) | SF switch |
| FLTMODE_CH / FLTMODE1–6 | 9 / STABILIZE, ALT_HOLD, LOITER, LAND, STABILIZE, STABILIZE | flight-mode switch (set by the team 2026-11-17; not in the overlay file) |
| RTL_ALT | 300 (3 m) | default 15 m is far too high indoors |
| SR0_POSITION / EXTRA1 / RC_CHAN / others | 10 / 10 / 5 / 2 Hz | MAVROS needs streams over USB (0 = heartbeat only, no RC, no position) |
| WPNAV_SPEED / WPNAV_ACCEL | 1000 / 250 (defaults) | **kept at default**: the fast square flies at the Cube's default speed |
| FS_EKF_ACTION / THRESH | 1 / 0.8 (read back 2026-11-09) | EKF failsafe → LAND |
| BATT_LOW_VOLT | 10.5 | **wrong for 6S**; open item for the EE team |

## Things the Cube does not announce

- `MAV_CMD_SET_EKF_SOURCE_SET` from the Orin is accepted silently (no "Using EKF Source Set" text; that only
  comes from the RC switch). The mission publishes its own record on `/uav/ekf_source_cmd`. After a commanded
  switch, SG disagrees with the Cube until the pilot flips it.
- `MAV_CMD_DO_CHANGE_SPEED` in GUIDED lasts until GUIDED's position control restarts.
