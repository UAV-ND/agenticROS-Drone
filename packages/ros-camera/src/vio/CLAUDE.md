# UAV-ND GPS-denied navigation: context for Claude Code

You are working on the Jetson Orin Nano of the UAV @ Notre Dame (UAV-ND) drone for the
Raytheon AVC 2026–27. Jonathan leads the software team and owns GPS-denied flight.
Preston (GPS-denied/VIO track) did the earlier work. Also read the older running log at
`~/ov_ws/CLAUDE uav.md` (note the space in the name) before starting.

## Current priority (as of Tue Nov 3, 2026)

Get **VIO on the RealSense D435i ready**: verified camera config, an estimator that holds on
recorded bags, then the live and vibration gates below. Deadlines: VIO go/no-go Wed Nov 4 night,
ICD sign-off Fri Nov 6, Sprint 3 Week 1 due Sun Nov 8.

## System architecture

- **Primary GPS-denied source (EKF3 source set 1):** ARK Flow (optical flow + rangefinder) on
  the Cube Orange's CAN1 via DroneCAN, straight into ArduCopter EKF3. No Jetson in the loop.
- **Secondary source (set 2):** D435i → Jetson VIO (OpenVINS) → bridge node (applies the
  camera-to-body transform incl. 20–30° down tilt) → MAVROS `/mavros/odometry/out` → MAVLink
  ODOMETRY → EKF3 ExternalNav. Set 2 flies only after VIO passes its gates.
- Flight controller: **Cube Orange running ArduCopter** (version TBD; 4.6 renamed some params).
- Airframe: Tarot 650, 6S, X4110S KV400 motors. RFD900x telemetry, FrSky X8R receiver.
  No GPS/compass mast on the EE diagram.
- The EE block diagram swaps two labels: ARK Flow is the flow + range sensor, and the D435i is
  the VIO camera.

## Machine

- Jetson Orin Nano, **Ubuntu 24.04, ROS 2 Jazzy, JetPack 7 (R39)**, 15 W mode.
- This differs from locked team decision G2 (JetPack 6.2 / 22.04 / Humble). Don't try to fix
  it; it's flagged to the deputy lead.
- Two OpenCV versions on the system: JetPack's 4.8 and Ubuntu's 4.6 (pulled in via cv_bridge).
- Workspaces:
  - `~/ov_ws`: OpenVINS, with the Jazzy port in `~/ov_ws/jazzy_fixes.patch`.
  - `~/rs_ws`: realsense-ros wrapper.
- librealsense 2.58.4 is built from source with the RSUSB backend.
- Access: people SSH in (VS Code Remote-SSH from Windows). **GUIs (RViz, rqt) only work on the
  Jetson's own monitor.** Never launch GUI tools over SSH; ask the human to run them.

## Key files

| Path | What it is |
|---|---|
| `~/vio_config/d435i_vio.yaml` | RealSense config. Always launch the camera with it: `ros2 launch realsense2_camera rs_launch.py config_file:=$HOME/vio_config/d435i_vio.yaml` |
| `~/vio_config/openvins_d435i/` | **Base** OpenVINS config (estimator, imucam, imu yamls). Never edit in place. |
| `~/vio_config/experiments/` | Config variants. A–J already tried; K, L, M come from `make_ov_experiments.py`. |
| `~/vio_config/run_ov_bag.sh <bag>` | Headless bag replay through OpenVINS with a trajectory summary. Env vars: `OV_CONFIG`, `OV_ARGS`, `PLAY_ARGS`, `TAG`, `DOMAIN`. Output in `~/vio_runs/`. Runs on its own ROS_DOMAIN_ID so a live camera can't contaminate it; keep it that way. |
| `~/vio_config/run_g6_live.sh` | Camera + OpenVINS + RViz live. Jetson monitor only. Starts the camera itself, so close any other camera launch first. |
| `~/record_g5.sh` | Records the 5 camera topics (camera must already be running). |
| `~/check_bag.py [bag]` | Rates, drops, stale frames. |
| `~/vio_config/tools/vio_triage.py` | Read-only triage of a bag + OpenVINS config: extrinsic convention, stereo pairing, image timing, IMU stream, IMU at rest vs config noise, image quality. Writes `report.txt` + sample PNGs. |
| `~/vio_config/tools/rs_check.py` | Live check of the running camera: USB 3 link, the node's *actual* params (`ros2 param dump`), topic rates, stereo pairing, IMU stamps, accel at rest, projector-dot count. |
| `~/vio_config/tools/make_ov_experiments.py` | Builds K (IMU noise ×10), L (`up_msckf_chi2_multipler` 5) and M (both) from the base config. Refuses to overwrite without `--force`. |
| `~/vio_config/tools/uavnd_gps_denied_stage1.parm` | ArduCopter params for ARK Flow (stage 1) with the VIO stage commented out. Humans load it in Mission Planner. |

Topic names are the wrapper defaults: `/camera/camera/infra1/image_rect_raw`,
`/camera/camera/infra2/image_rect_raw`, `/camera/camera/imu`, plus the camera_info topics.

## Camera and OpenVINS settings already established

Camera (`d435i_vio.yaml`):
- **IR emitter off.** It only takes effect from the config file; the command-line arg
  `depth_module.emitter_enabled:=0` is silently ignored.
- Accel at 250 Hz. At the default 63 Hz, about 70% of united IMU samples repeated an accel value.

OpenVINS base config:
- Intrinsics from camera_info: fx = fy = 421.631, cx = 423.190, cy = 238.556, 848×480,
  rectified, zero distortion.
- Extrinsics from the driver TF: R = I, IMU position in the infra1 optical frame =
  [0.00552, −0.0051, −0.01174] m, stereo baseline 50.068 mm.
- Lean settings: stereo, 150 KLT points, max_slam 0, 2 threads.
- `track_frequency` 31, because 30 drops about half the frames.
- `init_imu_thresh` 0.3, because 1.5 never fires on a gentle pickup.

## Bags (`~/bags/`): never delete or overwrite

| Bag | Notes |
|---|---|
| `rs_g5_1015_0041` | Oct 15, 66 s, aggressive hand-waving. Not usable for VIO. |
| `rs_g5_1102_1834` | 202 s: still ~45 s, then picked up with a 180° flip. One 0.6 s gap. |
| `rs_g5_1102_1841` | 113 s. Still period stares at a blank tabletop, so static init never fires. |
| `rs_g5_1102_1958` | **Best test bag.** 62 s: upright and still, firm shake, slow sideways walk. Zero drops. |
| (to record) loop bag | Starts and ends on a taped X after a 10 m walk, so the final position = drift. |

## The open problem: OpenVINS diverges on every D435i bag

Symptom:
- OpenVINS initializes correctly, then position runs away (kilometres within a minute).
- 80–95% of MSCKF features fail triangulation, and chi2 rejects about half the rest, so the
  filter runs nearly IMU-only.

Tried without success (variants A–J):
- max_slam 50
- online calibration off
- EuRoC params
- fi_max_baseline 200
- fi_max_cond_number 50000
- dynamic init
- mono

Verified correct on the data:
- Camera-IMU time lag is 0 ms (gyro vs. vision-rotation cross-correlation).
- Gyro axes match the camera axes (R = I).
- Accel agrees with gyro-propagated gravity to within 2–4°.
- Right camera is at +x, with rows aligned to within 0.1 px.
- OpenVINS's startup print shows the loaded intrinsics/extrinsics.
- Left and right images reaching OpenVINS differ correctly.

Control: the same build works on EuRoC V1_01, though it also fails 63% of triangulations there.

How to read the symptom: OpenVINS triangulates with its own estimated poses. Once the state
drifts, triangulation fails, which drifts it further. So the failure counts are mostly a
consequence. Look at the first seconds after init.

Untested hypotheses, in priority order:
1. **IMU consistency.** D435i IMUs ship poorly calibrated, and 2–4° of accel-vs-gyro gravity
   disagreement is a red flag, not a pass. 3° leaks ~0.5 m/s² of gravity into the horizontal,
   which is ~900 m in 60 s. IMU noise was never inflated in A–J.
   - Test: variants K/M.
   - Fix: Intel's rs-imu-calibration (needs pyrealsense2; ROS camera stopped).
2. **Extrinsic convention.** OpenVINS's `T_imu_cam` is camera→IMU (R_CtoI, p_CinI); Kalibr's
   `T_cam_imu` is the inverse. The cam1 origin expressed in the cam0 frame must come out
   ≈ [+0.050, 0, 0]. If it's −0.050, the baseline is inverted.
   - Test: `vio_triage.py` check 1.
3. **Stereo pairing / timestamps.** OpenVINS pairs left and right with an approximate-time
   sync, so an orphan frame gets paired with a neighbor 33 ms away.
   - Test: triage checks 2–3 and `rs_check.py`.
4. **Exposure, blur, emitter.** Test: triage check 6.
5. Only after those: the two-OpenCV ABI mix, Jazzy build differences, and 848-wide images
   (try a 640×480 infra profile with the new camera_info intrinsics).

## Plan (work through in order, report after each step)

1. **Camera check.** Ask the human to launch the camera with `d435i_vio.yaml`. Then run
   `python3 ~/vio_config/tools/rs_check.py` while they hold the camera still and upright for the
   first 2 s. Fix FAILs in `d435i_vio.yaml` (ask first) and recheck. If a fix changes the stream
   (sync, unite method, emitter), old bags still carry the old problem.
2. **Triage:** `python3 ~/vio_config/tools/vio_triage.py ~/bags/rs_g5_1102_1958`.
3. **Loop bag.** Ask the human to record it:
   1. Tape an X on the floor and set the camera on it, upright.
   2. Hold still 5 s, then pick it up gently.
   3. Walk a 10 m loop through a lit, cluttered room (avoid blank walls).
   4. Put it back on the X and hold still 5 s.
4. **Experiments.** Run `python3 ~/vio_config/tools/make_ov_experiments.py`, then
   `run_ov_bag.sh` for K, L and M on both bags. First read `run_ov_bag.sh` to find out whether
   `OV_CONFIG` takes the folder or the `estimator_config.yaml` path; match how A–J were run.
5. **Decide:**
   - Triage says baseline inverted → invert both 4×4s in a new variant (not the base) and rerun.
   - K or M holds → IMU consistency:
     1. Human runs rs-imu-calibration.
     2. Re-record the loop bag.
     3. Step noise down (×5, ×3) to the lowest value that holds, then keep 2× margin.
   - Only L holds → chi2 is rejecting good vision. Usually the same root cause; tune as above.
   - Nothing holds and a pairing/stamp check FAILs → fix camera/driver config, re-record.
   - Nothing holds and all checks PASS → OpenVINS/build problem. Recommend cuVSLAM (G9) to
     Jonathan, after checking that Isaac ROS supports the Orin Nano on JetPack 7. Don't install it.
6. **Gates for "VIO ready":**
   1. Holds on rs_g5_1102_1958 end to end, shake included.
   2. Loop bag ends within about 0.5 m of the X.
   3. Same loop live with `run_g6_live.sh` (human, Jetson monitor).
   4. Camera mounted, drone tied down, motors running: bag recorded and replayed without
      divergence (vibration).

After VIO is ready, the next pieces are:
- The bridge node: camera-to-body transform incl. tilt; publish nav_msgs/Odometry ENU/FLU on
  `/mavros/odometry/out`; stop publishing when VIO is uninitialized or its covariance is too
  high; never resend a stale pose.
- MAVROS on TELEM2 at 921600 (TBD).
- Measuring VIO latency for VISO_DELAY_MS.

## Rules

Ask Jonathan first before:
- installing large software (Isaac ROS/cuVSLAM, Docker images);
- running apt upgrade, reflashing, or changing JetPack;
- editing `d435i_vio.yaml` or anything in `~/rs_ws` or `~/ov_ws` source;
- deleting any file.

Never:
- Edit `~/vio_config/openvins_d435i/`. New configs go in `~/vio_config/experiments/<letter>/`.
- Leave OpenVINS source changed. It must match `jazzy_fixes.patch`. Temporary debug prints are
  fine, but remove them and rebuild afterwards, and say so in the log.
- Write flight controller parameters or send commands to the FC from the Jetson. FC changes are
  made by a human in Mission Planner.
- Launch GUI tools over SSH.

Camera handling:
- Only one process can own the camera. Close other camera launches before `run_g6_live.sh`.
- Stop a recording with Ctrl+C and wait for the summary. If it's stuck, run
  `pkill -INT -f "ros2 bag record"`. **Never `kill -9`**: it corrupts the bag.
- Bags are named by the minute, so wait for a new minute before starting another recording.
- A few `control_transfer returned error ... number: 11` lines at camera startup are harmless
  if the node comes up.
- After a reboot nothing runs by itself: camera launch first (terminal 1), then recording
  (terminal 2).

General:
- ROS commands like `ros2 topic hz` / `echo` run forever: wrap them in `timeout`.
- Physical steps (moving the camera, walking a loop, motors, the monitor) are done by a human.
  Tell them exactly what to do, then wait.
- After each experiment, append a dated entry to `~/ov_ws/CLAUDE uav.md`: what ran, which
  config, which bag, and the result.

## Open items (not for Claude Code to decide)

- ArduCopter version on the Cube.
- TELEM port and Jetson UART device.
- ARK Flow and D435i offsets from CG, and the D435i tilt angle.
- RC channel for the EKF source switch.
- Whether the Cube's internal compass is usable.
- AVC GPS-denied task wording, which sets the gate thresholds.

ICD draft (humans only):
https://claude.ai/artifact/MGyuT1VdGAV9fxYfMMwPZB
