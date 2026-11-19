# UAV-ND GPS-denied: VIO → flight controller integration report

**Dates:** Nov 5 2026 (evening) – Nov 6 2026 (early morning)
**Machine:** Jetson Orin Nano Dev Kit Super (JetPack 7 / L4T R39.2.1, Ubuntu 24.04, ROS 2 Jazzy)
**Flight controller:** Cube Orange, **ArduCopter V4.6.3** (92b0cd78), QUAD/X
**Branch:** `CV` (all new files uncommitted at time of writing)

---

## 1. Summary

Starting point: cuVSLAM VIO had a *conditional go* (Nov 4), but nothing connected it to the flight controller.

By the end of this session the full GPS-denied chain was built and **verified on the bench**:

```
D435i ─► cuVSLAM (VIO, Docker) ─► vio_mavros_bridge (new) ─► MAVROS ─► USB ─► Cube Orange ─► EKF3 (source set 2)
                                                                                   ▲
                                                ARK Flow (CAN) ───────────────────┘ (source set 1)
                                                RC ch10 switch selects set 1 / set 2
```

Headline results:

| Result | Evidence |
|---|---|
| **EKF3 accepts and fuses the VIO** | Cube log: *"Using EKF Source Set 2"*, *"EKF3 IMU0/1/2 is using external nav data"*; EKF position matched VIO to **~1 cm mean (±2–3 cm)**; innovation test ratios ≤ 0.48 (reject at 1.0) |
| **Bridge rejects cuVSLAM jumps** | On the occlusion bag: drift **3.99 m → 0.58 m** (VIO) and **50.7 m → 0.46 m** (stereo); clean runs untouched |
| **Real-time on the Orin** | 30 Hz end to end, cuVSLAM latency median 19 ms / p95 24 ms, CPU ~20 %, GPU ~12 %, ~5.8 W |
| **Live room-map visualization** | cuVSLAM landmark map + paths in RViz while the pipeline runs |

Main open hardware issue: **the 40-pin UART link only works one way** (Cube → Orin). USB is used instead.

---

## 2. Orin ↔ Cube Orange link

### 2.1 Planned wiring (UART, TELEM2)

| Cube TELEM2 (JST-GH) | Orin J12 (40-pin) |
|---|---|
| pin 2 TX | pin 10 UART1_RX |
| pin 3 RX | pin 8 UART1_TX |
| pin 6 GND | pin 6 GND |
| pin 1 +5 V | **not connected** |

Orin device: **`/dev/ttyTHS1`** (`serial@3100000`, UARTA). Pinout charts list `/dev/ttyTHS0`, which is the old JetPack name; JetPack 7 numbers it `ttyTHS1`.

### 2.2 The UART fault (unresolved)

- **Cube → Orin works.** ArduPilot HEARTBEAT received cleanly at 57600 (the Cube's default; `SERIAL2_BAUD` was later set to 921).
- **Orin → Cube never works.** 0 replies to TIMESYNC, read-only PARAM_REQUEST_READ, and MAVROS requests.

Ruled out:

| Check | Result |
|---|---|
| Baud / protocol / sender sysid | 57600 and 921600; MAVLink 1 and 2; sysid 1.191 and 255.190; no replies |
| Cube port settings | `SERIAL2_PROTOCOL` 2, `SERIAL2_OPTIONS` 0 (no half-duplex/invert), `BRD_SER2_RTSCTS` later 0 |
| Orin pinmux | pins 8/10 = `uarta`; pinconf: SFIO mode, TX `tristate=0` (driver on) |
| Orin driver | `nvidia,tegra194-hsuart`; 5760 B drained in 1.00 s at 57600 (expected 1.0 s); no CRTSCTS; no getty on the port |
| Wires | 3 different wires; multimeter: 3.3 V idle at both ends of the TX wire |
| Cube port | Same failure on **TELEM1** and **TELEM2** |

**Not yet done:** a loopback test (Cube unplugged, jumper J12 pin 8 ↔ 10, `uart_loopback.py`). This splits "Orin pin 8 not transmitting" from "signal not reaching or being accepted by the Cube".

### 2.3 Workaround: USB (working)

- Cube micro-USB → Orin: `/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00` (`ttyACM0`)
- TIMESYNC 5/5 replies, round trip 2–5 ms; parameter reads OK; MAVROS connected.
- **Orin USB hub port `1-2.4` went bad** during the session (enumeration errors −71 / −110). The Cube works on the laptop and on another Orin port. **Avoid that port.**
- Trade-off: Mission Planner can't share the USB port, so the ground station should use the RFD900.

---

## 3. Software built

All new files are in `packages/ros-camera/src/vio/` unless noted.

### 3.1 VIO → MAVROS bridge

| File | Purpose |
|---|---|
| `vio_bridge_core.py` | Frame math and safety gating (pure Python, no ROS) |
| `vio_mavros_bridge.py` | rclpy node: `/visual_slam/tracking/odometry` → `/mavros/odometry/out` (+ `/vio_bridge/status` JSON at 1 Hz) |
| `vio_bridge.yaml` | Parameters (mount, gates, leveling) |
| `test_vio_bridge_core.py` | 10 unit tests (all pass) |
| `bridge_replay_check.py` | Offline replay of recorded cuVSLAM runs through the gating |

**Frames.**
- **Input:** cuVSLAM pose of `camera_link` in its own `odom` frame (= camera pose at start, tilted ~30° down). The twist is in the child frame (verified against finite differences).
- **Output:** pose of the body (`base_link`, FLU) in a gravity-aligned world, plus body-frame twist. The world is either the body pose at cuVSLAM start (`level_mode: none`) or leveled from the D435i accelerometer (`camera_imu`, unit-tested only).
- **Mount:** applied once in the bridge (`cam_x/y/z`, `cam_pitch_down_deg` = 30 placeholder), so ArduPilot's `VISO_POS_*` stays 0.
- **MAVROS:** frame IDs `odom` / `base_link`. MAVROS converts to MAVLink ODOMETRY (LOCAL_FRD / BODY_FRD) with its own static `_ned` / `_frd` TFs.

**Gating.** Needed because cuVSLAM reported "tracking OK" through occlusion jumps of up to 37 m.
- **Stale stamps:** stale, duplicate or out-of-order stamps are dropped and never re-sent. Input older than 0.25 s is dropped.
- **Jumps** (implied speed > 4 m/s, rotation > 300°/s, twist > 4 m/s): the bridge dead-reckons across the step and re-anchors. The published track stays continuous, because MAVROS never sets `reset_counter`.
- **Input gaps:** output pauses, and the motion across the gap is kept if it's plausible.
- **High covariance** (> 0.02 m²): output pauses, with no re-anchor.
- **Resume:** output restarts after 15 consecutive good frames. While output is off, EKF3 falls back to set 1.
- Thresholds come from the recorded data: normal motion ≤ 0.5 m/s and ≤ 50°/s; jumps 4–1100 m/s and up to 2300°/s.

**Replay results on recorded cuVSLAM runs:**

| Run | Raw start→end | Bridged | Output on |
|---|---|---|---|
| Occlusion loop, VIO (`rs_g5_1104_2016` m1) | 3.99 m | **0.58 m** (39 jumps absorbed) | 98.0 % |
| Occlusion loop, stereo (m0) | 50.7 m | **0.46 m** | 98.1 % |
| Clean loop2, VIO / stereo | 0.039 / 0.085 m | unchanged, 0 rejects | 99.5 % |
| Live-check, VIO | 0.029 m | unchanged, 0 rejects | 99.4 % |
| Nov 3 X-B-X, VIO | 0.92 m | **0.28 m** | 98.8 % |
| Scale bag (output gaps), VIO | 0.613 m | 0.611 m (gaps kept) | 97.7 % |

The ROS node gave identical numbers to the offline core when the occlusion bag was replayed through it.

### 3.2 MAVROS and link tools

| File | Purpose |
|---|---|
| `mavros_telem2.launch.py` | MAVROS (+ `bridge:=true`). Default `fcu_url` = the Cube's USB by-id path. Plugin allowlist: `sys_status, sys_time, imu, local_position, odometry, rc_io`. No setpoint/command/param/mission plugins |
| `mavros_params.yaml` | `/mavros/time`: `timesync_rate` 10 Hz (default is 0 = off without apm_config) |
| `install_mavros.sh` | Installs `ros-jazzy-mavros` + `mavros-extras` 2.16.0 + GeographicLib datasets (human ran it) |
| `mav_link_check.py` | Listen-only link check (heartbeat, message rates, decode errors) |
| `uart_loopback.py` | Orin UART loopback test (jumper pin 8 ↔ 10) |
| `~/mav_venv` | pymavlink 2.4.50 venv (PEP 668 blocks system pip) |

Bugs found and fixed during bring-up:
1. **Topics doubled to `/mavros/mavros/...`:** `mavros_node` names itself `mavros`, so the launch namespace was removed.
2. **Time sync silently off:** fixed with `mavros_params.yaml`. Result: 10 Hz, 3.5 ms round trip.
3. **rclpy logger crash:** one call site can't switch severity; fixed in the bridge.

### 3.3 Map visualization (in `~/vio_config/`)

| File | Purpose |
|---|---|
| `cuvslam/cuvslam_live_viz.launch.py` | Copy of the live launch with `enable_slam_visualization`, `enable_landmarks_view`, `enable_observations_view`, `path_max_size` 8192. Nothing else differs |
| `run_cuvslam_live.sh` | New `LAUNCH=` option (default unchanged; backup `run_cuvslam_live.sh.bak_1106`) |
| `cuvslam/cuvslam_map.rviz` | RViz layout: landmark map (coloured by height), loop closures, SLAM path, VIO path, camera arrow, TF, IR image |

Run with:
```
LAUNCH=cuvslam_live_viz.launch.py MODE=1 bash ~/vio_config/run_cuvslam_live.sh
rviz2 -d ~/vio_config/cuvslam/cuvslam_map.rviz        # on the Jetson monitor
```

The landmark map publishes about every 10 s (578 points on the first check), and the loop-closure cloud showed 136 points.

---

## 4. Flight controller configuration

All parameter changes were made by a human in Mission Planner. Per project rules, the Jetson does not write FC parameters. Values below were read back from the Cube (read-only).

| Group | Final values |
|---|---|
| Source set 1 (ARK Flow) | `EK3_SRC1_POSXY` 0, `EK3_SRC1_VELXY` 5, `EK3_SRC1_VELZ` 0, `EK3_SRC1_POSZ` 1, `EK3_SRC1_YAW` 1 |
| Rangefinder / flow | `EK3_RNG_USE_HGT` 70, `RNGFND1_TYPE` 24 (DroneCAN), `RNGFND1_MIN_CM` 8, `FLOW_TYPE` 6, CAN1 DroneCAN |
| Source set 2 (VIO) | `EK3_SRC2_POSXY` 6, `EK3_SRC2_VELXY` 6, `EK3_SRC2_POSZ` 1, `EK3_SRC2_VELZ` 0, `EK3_SRC2_YAW` 6, `EK3_SRC_OPTIONS` 0 |
| Visual odometry | `VISO_TYPE` 1, `VISO_ORIENT` 0, `VISO_POS_X/Y/Z` 0, `VISO_DELAY_MS` 30 (measured: median 19 ms, p95 24 ms) |
| Source switch | **`RC10_OPTION` 90**, `RC9_OPTION` 0 (ch9 is not mapped on the transmitter) |
| Logging | `LOG_DISARMED` 1 |
| TELEM2 | `SERIAL2_PROTOCOL` 2, `SERIAL2_BAUD` 921, `BRD_SER2_RTSCTS` 0 |
| GPS | `GPS1_TYPE` 0 |

Notes:
- **The stage-1 parm file had only partly been loaded.** Source set 1 was still on GPS (3/3/3) with GPS disabled, which left the EKF in constant-position mode. This is now fixed.
- **"PreArm: VisOdom: out of memory"** means the visual-odometry module wasn't created at boot. A **full power-cycle** after `VISO_TYPE` 1 fixed it.
- **The parm file uses 4.6-renamed names that this firmware doesn't have:** `RNGFND1_MIN/MAX` (it uses the `_CM` versions) and `GPS_TYPE` (it uses `GPS1_TYPE`). The file should be updated.
- **`RC10_OPTION` 158** (flow calibration) was planned for ch10 in the parm file and now needs another channel.
- **`SR0_*` stream rates are 0**, so over USB the Cube sends only HEARTBEAT to the Orin. Live monitoring from the Jetson needs them set by a human.

Hardware added during the session: **ARK Flow** (CAN; flow healthy, rangefinder healthy above 8 cm) and the **FrSky X8R receiver** (16 channels).

---

## 5. Bench tests

### 5.1 Full chain, no fusion (Nov 5, 23:42; `VISO_TYPE` 0)

| Stage | Result |
|---|---|
| cuVSLAM | 29.6 Hz, 0 gaps, latency median 19 / p95 24 / max 68 ms; CPU 20 %, GPU 12 %, 5.8 W |
| Bridge | 29.7 Hz, stamps strictly increasing, max gap 67 ms; 3615/3629 published (14 = init); 0 rejects |
| MAVROS | connected, no frame/TF errors |
| Motion | handheld ~6 m, 212° yaw, ended 19.6 cm from start (start not taped; cuVSLAM raw 18 cm) |

### 5.2 EKF fusion test (Nov 6, ~01:08; props off)

Procedure: ch10 low → middle (set 2) → move the camera ~0.5–0.7 m and back → low. The analysis used the Cube's own DataFlash log.

| Item | Result |
|---|---|
| Switch | `RCIN.C10` middle 273.4–328.5 s (log time) |
| Cube messages | *Using EKF Source Set 2* → *EKF3 IMU0/1/2 is using external nav data*, yaw aligned → *Using EKF Source Set 1* |
| VIO received | `VISP`/`VISV` at 30 Hz (1649 samples in 55 s) |
| Position alignment | At the switch the EKF reset horizontal position to the VIO's within 0.4 s |
| Tracking | EKF − VIO: mean N −1.1 cm, E −0.7 cm, std 2–3 cm; relative-motion error median 0.8 cm, p95 8 cm, max 23 cm; correlation 0.98 |
| Consistency | `XKF4` test ratios on set 2: position max 0.09, velocity max 0.32, height max 0.48 (1.0 = reject) |
| Height | EKF height followed the ARK rangefinder (1.40 m, facing something 1.4 m away). By design VIO isn't used for height |

**Conclusion: the full chain works and EKF3 fuses the VIO; switching between set 1 and set 2 works.**

### 5.3 Room map demo

The cuVSLAM landmark map, paths and camera pose display in RViz while the pipeline (and optionally the bridge → Cube) runs. This is a visual sanity check of tracking quality, not a quantitative test.

---

## 6. Open items and next steps

**Before flight:**
1. **Mount the D435i and ARK Flow on the airframe.** Measure the camera offset from CG and the down-tilt, and set them in `vio_bridge.yaml` (`cam_*`; the 30° and zero offsets are placeholders).
2. **Vibration test:** motors on, drone tied down; check accel peaks/clipping and replay the bag.
3. **Tethered / low hover:** switch between set 1 and set 2 in the air; watch EKF status.
4. **Check `max_speed` (4 m/s)** against the AVC task's flight speeds.
5. **Decide on `level_mode`** (`none` needs the drone level when cuVSLAM starts).

**Link and config:**

6. **UART:** run the loopback test to locate the fault, or formally adopt USB in the ICD (secured cable with strain relief).
7. **Set `SR0_*`** (`RC_CHAN` 5, `EXT_STAT` 2, `POSITION` 10, `EXTRA1` 10, `EXTRA3` 2) for live monitoring from the Jetson.
8. **Update `uavnd_gps_denied_stage1.parm`:** firmware-correct names (`GPS1_TYPE`, `RNGFND1_*_CM`), the source switch on RC10, a new channel for flow calibration, `VISO_DELAY_MS` 30.
9. **Avoid Orin USB hub port `1-2.4`.**

**Housekeeping:**

10. **Commit** the new files on `CV`.
11. **Docker:** the cuVSLAM container once took ~70 s to start (stuck in *Created*). Monitor it.
12. **IMU calibration** and the 10 m+ loop from the Nov 4 plan are still pending.

### Suggested ICD wording

> Companion link: Jetson Orin Nano ↔ Cube Orange via **USB (CDC-ACM), MAVLink 2**, MAVROS 2.16 (ROS 2 Jazzy). VIO delivered as MAVLink ODOMETRY (`/mavros/odometry/out`, ~30 Hz) to ArduPilot VISO_TYPE 1, EKF3 source set 2, selected by RC ch10 (`RC10_OPTION` 90). TELEM2 UART (921600) is the intended path pending a hardware fix.

---

## 7. Where things are

| What | Where |
|---|---|
| New code and tools | `packages/ros-camera/src/vio/` (this repo, branch `CV`) |
| cuVSLAM launch/runner/RViz | `~/vio_config/`, `~/vio_config/cuvslam/` |
| Running log | `~/ov_ws/CLAUDE uav.md` (dated entries Nov 5–6) |
| Bench recordings | `~/vio_runs/bridge_bench_*`, `~/vio_runs/fusion_bench_*`, `~/vio_runs/live_cuvslam_m1_{bridge,fusion,fusion2,map}_*` |
| Cube DataFlash log of the fusion test | `~/Downloads/1 12-31-1969 7-00-00 PM.bin` (the two downloaded files are identical) |
