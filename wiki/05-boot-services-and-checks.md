# 05 Boot services and checks

[← wiki home](README.md)

Files: `ros2_ws/src/uav_mission/systemd/`. The human installs and restarts services (`sudo`); agents print the commands.

## Services

| Service | Runs | Notes |
|---|---|---|
| `uav-camera` | `start_camera.sh`: realsense2_camera with `~/vio_config/d435i_vio.yaml` | USB/duplicate checks |
| `uav-cuvslam` | `start_cuvslam.sh`: cuVSLAM Docker | restarts with the camera |
| `uav-mission` | `start_mission.sh`: MAVROS + bridge + fc_status + mission + ch6 trigger + recorder; Cube-replug watchdog; `Restart=always` | **installed at boot** since 2026-11-18 |
| `uav-fclink` | `start_fclink.sh`: the same link **read-only** (no mission, can't arm) | conflicts with `uav-mission` |

```bash
sudo bash .../systemd/install_services.sh mission   # mission at every boot (disables fclink)
sudo bash .../systemd/install_services.sh fclink    # manual flying only (disables mission)
sudo systemctl start uav-fclink                     # read-only for this power-on only (stops uav-mission)
sudo systemctl restart uav-mission                  # reload code / mission file (disarmed only!)
systemctl status uav-mission;  journalctl -u uav-mission -f
```

Environment for all scripts: `uav_env.sh` (ROS domain, workspace, `UAV_MISSION_PARAMS` from `~/.uav_mission`).

## Health checks (read-only, safe any time)

| Script | PASS means |
|---|---|
| `check_camera.sh` | camera streaming at ~30 fps |
| `check_cuvslam.sh` | cuVSLAM ≥ 20 Hz |
| `check_fclink.sh` | MAVROS connected, Cube attitude/position ≥ 5 Hz, RC seen, bridge TRACKING, VIO to Cube ≥ 20 Hz; prints mode, EKF set, switch positions |
| `check_mission.sh` | mission node IDLE, preflight would accept a start now, ch6 trigger ready; prints the mission file |
| `check_all.sh` | all four → `ALL READY` |

## Logs and recordings (`~/flight_logs/`)

- `boot####_<date>_<id>/`: per-boot service logs (`camera.log`, `cuvslam.log`, `mission.log`).
- `run####_<date>_mission/` (or `_fclink`): rosbag (mcap) of MAVROS, VIO, mission state, RC, battery for that power-on.
  **Run `sudo poweroff` before pulling the battery**, or the bag is left unindexed (fix: `ros2 bag reindex`).
- Analysed flights: `2026-11-18_outdoor_vio_flight/`, `first_autonomous_square_run0004/` (with `analyze_square.py`).
- Dates read `1969...` when the Orin had no network time; go by the run/boot number.

## Reaching the Orin

- **USB-C**: `ssh uav-nd-orin@192.168.55.1`. The L4T DHCP lease is 15 s and Windows doesn't renew it →
  give the laptop's USB adapter a **static 192.168.55.2/24** (no gateway).
- **Wi-Fi** (eduroam): DHCP, address from `hostname -I` on the Orin; changes.
- **Bluetooth PAN**: `orin_setup/bluetooth_ssh/` (169.254.77.1). Written, not yet installed.
- Cube USB: use port 1-2.4 (1-2.3 failed to enumerate). ModemManager is disabled (it grabbed the Cube's port).
  Don't send `reboot` to the Cube over MAVLink (USB lockup once); replug instead. The mission service
  notices a replug and restarts the link within ~10 s.
