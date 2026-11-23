# 02 Repository map

[← wiki home](README.md)

Repo: `github.com/UAV-ND/agenticROS-Drone`. On the Orin: `~/Documents/uav-nd/agenticROS-Drone`.

This repo is a fork of **Corebrum Synapse** (ROS 2 ↔ AI agent bridge for ground robots). Most top-level
docs (`README.md`, `CLAUDE.md`, `AGENTS.md`, `docs/`, `packages/synapse-*`, `ros2_ws/src/synapse_*`) describe
that upstream project and its ground robot (`/cmd_vel`, Zenoh, OpenClaw). **They do not describe the drone.**
Synapse is only used to let an AI agent start/abort missions and read status.

## Drone code (UAV-ND)

| Path | What |
|---|---|
| `ros2_ws/src/uav_vio/` | VIO → Cube: `vio_mavros_bridge` (camera→body mount, gating), `fc_status` (read-only monitor), `fc_link.launch.py`, `bridge_replay_check`. Config `config/vio_bridge.yaml`. [03](03-vio-pipeline.md) |
| `ros2_ws/src/uav_gnc/` | `FcCommander`, the **only** code that commands the Cube (mode, arm, takeoff, setpoints, speed, EKF source), clamped by `limits.py`. `guidance.py` = PID / path helpers |
| `ros2_ws/src/uav_mission/` | Mission state machine. `uav_mission/mission_core.py` (pure Python, unit-tested), `mission_node.py` (ROS wrapper), `rc_trigger_*` (ch6 start/abort), `fake_die.py` |
| `ros2_ws/src/uav_mission/config/` | Mission files: `mission.yaml` (defaults), `square_3m_vio.yaml`, `square_10m_fast_vio.yaml`, `flight_test_vio.yaml` (older hover/square/sweep test) |
| `ros2_ws/src/uav_mission/systemd/` | Boot services, start scripts, `check_*.sh` health checks, `install_services.sh`, `select_mission.sh`, `uav_env.sh`. [05](05-boot-services-and-checks.md) |
| `ros2_ws/src/uav_mission/sitl/` | Simulator kit: `sitl_gps_denied.parm`, `rc.py` (fake transmitter), `rec.py`, README. [06](06-simulation.md) |
| `ros2_ws/src/uav_mission/test/`, `uav_gnc/test/`, `uav_vio/test/` | Unit tests (pytest, no ROS graph needed) |
| `packages/ros-camera/src/vio/` | VIO bring-up tools and Cube settings: `cube_gps_denied.parm`, `cube_known_good_2026-11-09.parm`, `check_cube_params.py`, `rs_check.py`, `vio_triage.py`, IMU tools, `REPORT_2026-11-06_vio_to_flight_controller.md`, an older `CLAUDE.md` (OpenVINS era, partly outdated) |
| `packages/synapse-drone-claude-code/` | **Drone MCP tools for AI agents** (`drone_takeoff`, `drone_navigate`, `drone_search`, `drone_detect_dice`, `drone_abort`, `drone_get_status`, ...). On `main` only (PR #2). Not wired to `uav_mission` yet; see [12](12-open-items.md) |
| `packages/object-detection/` | Dice detection (YOLO, 4 colours): `detect.py`, `train.py`, `MANUAL.md` |
| `die_tracking/` | Teammates' die-tracking / height code (on `main` via PR #6; not on `mvp-mission`) |
| `mvp/` | MVP mission and architecture diagrams (v3 = ArduPilot version) |
| `orin_setup/bluetooth_ssh/` | SSH over Bluetooth PAN (written, not yet installed) |
| `wiki/` | This wiki |

## Outside the repo, on the Orin

| Path | What |
|---|---|
| `~/vio_config/` | Camera config `d435i_vio.yaml`, cuVSLAM launch scripts `run_cuvslam_live.sh` / `run_cuvslam_bag.sh`, cuVSLAM Dockerfile |
| `~/rs_ws`, `~/ov_ws`, `~/cvb_ws` | realsense-ros, OpenVINS (abandoned for cuVSLAM), vision_opencv workspaces |
| `~/ardupilot` + `~/mav_venv` | ArduCopter 4.6.3 SITL build and its Python venv |
| `~/flight_logs/` | Per-boot service logs (`boot####_*`), recordings (`run####_*_mission` / `_fclink`), analysed flights |
| `~/bags/`, `~/vio_runs/` | Camera bags and cuVSLAM/OpenVINS run results. **Never delete** |
| `~/.uav_mission` | Name of the mission the boot service flies (written by `select_mission.sh`) |

## Branches

- `main`: what's merged. `mvp-mission`: the drone software branch, PR'd into `main` feature by feature.
- Teammates' branches: `dice-tracking-sim` (PR #9), `drone-tools` (PR #3), `kwebb7/Drone-height` (merged #6).
