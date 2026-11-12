# uav_vio

GPS-denied VIO for the UAV-ND drone (ArduCopter 4.6 on Cube Orange, Jetson Orin Nano, ROS 2 Jazzy).

```
D435i ─► cuVSLAM (Docker) ─► vio_mavros_bridge ─► MAVROS ─► USB ─► Cube Orange ─► EKF3 source set 2
                                                    ARK Flow (CAN) ─────────────► EKF3 source set 1
                                                    RC ch10 (RC10_OPTION 90) selects the set
```

| Node | What it does |
|---|---|
| `vio_mavros_bridge` | `/visual_slam/tracking/odometry` → `/mavros/odometry/out`. Camera→body mount, gravity-aligned world, jump/gap/covariance gating (`config/vio_bridge.yaml`). Status JSON on `/vio_bridge/status`. |
| `fc_status` | Read-only monitor. `/uav/fc_status` JSON at 1 Hz: link, armed, mode, EKF source set (RC ch10 + Cube STATUSTEXT), bridge state, warnings (e.g. "set 2 selected but bridge in HOLD"). |
| `bridge_replay_check` | Offline: replay recorded cuVSLAM odometry bags through the bridge gating. |

## Bring-up (bench)

```bash
# 1. camera
ros2 launch realsense2_camera rs_launch.py config_file:=$HOME/vio_config/d435i_vio.yaml
# 2. cuVSLAM stereo-inertial (Docker; Ctrl+C once to stop, writes ~/vio_runs/...)
MODE=1 bash ~/vio_config/run_cuvslam_live.sh
# 3. MAVROS + bridge + status monitor
source ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/install/setup.bash
ros2 launch uav_vio fc_link.launch.py bridge:=true
# 4. watch
ros2 topic echo /uav/fc_status
```

`fc_link.launch.py` loads only read-only MAVROS plugins: nothing in this package can arm, change
mode or write FC parameters. FC parameters are set by a human in Mission Planner.

## Before flight (open items)

- Measure the D435i offset from CG and its down-tilt on the airframe; set `cam_*` in `config/vio_bridge.yaml` (current values are placeholders).
- `SR0_*` stream rates must be set (by a human) for `/mavros/rc/in`, local position etc. over USB.
- Vibration test (motors on, tied down), then pilot-flown Loiter on set 1 → switch to set 2 in the air.

## Tests

```bash
cd ros2_ws && colcon build --packages-select uav_vio --symlink-install && colcon test --packages-select uav_vio
```

Background: `packages/ros-camera/src/vio/REPORT_2026-11-06_vio_to_flight_controller.md` (repo root).
