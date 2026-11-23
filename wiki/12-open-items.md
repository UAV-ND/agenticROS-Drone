# 12 Open items and known issues

[← wiki home](README.md)

Update this list when something is done or found. Dates are when it was raised.

## Next up

| Item | Notes |
|---|---|
| Fly the fast 10 m square | Steps `path_speed` 1.0 → 2.0 → 0.0 (Cube default, ~3.8 m/s); check VIO logs between flights. Needs a ~16 × 16 m clear area. Checklist Part G |
| Competition mission (transit, sweep, centre on die, return to the start square) | `profile: sweep` written and unit-tested, not SITL-flown with a detector; needs the real field size and the die detector publishing `/uav/die/target` |
| Precise return to the 5 ft start square | VIO landed 3 cm from takeoff (EKF frame); ground truth not measured yet |
| Decide how AI agents drive the drone | `packages/synapse-drone-claude-code` (PR #2) has tools like `drone_takeoff` / `drone_navigate`, but the 2026-11-07 decision is that agents only start/abort `uav_mission` and read status. Reconcile: map the tools onto `/uav/mission/start|abort` and `/uav/mission/state` |
| Teammates' open PRs | #15 reverts #8 (drone height, `die_tracking/`); #9 dice centering (`dice-tracking-sim`); #3 mission-3 reroute helper (`drone-tools`). Update the wiki when they merge |
| Integrate dice detection with the mission | `packages/object-detection/`, PR #9 (`dice-tracking-sim`) |

## Open issues

| Raised | Issue |
|---|---|
| 2026-11-09 | **Battery failsafe not set for 6S** (BATT_LOW_VOLT 10.5, no critical action). EE team; until then crews watch time/voltage |
| 2026-11-09/17 | Cube parameters silently reverted twice; cause unconfirmed (old file in Mission Planner?). Check every flight day |
| 2026-11-13 | "Ground mag anomaly, yaw re-aligned" after every takeoff |
| 2026-11-18 | cuVSLAM variance briefly exceeds the 0.02 gate at takeoff/landing → sub-second bridge holds (tolerated) |
| 2026-11-18 | Baro height lags the rangefinder near the ground → LAND touches down faster than 0.5 m/s |
| 2026-11-18 | Cube IMU heater blocks arming ~2.5 min in the cold |
| 2026-11-19 | Cube `.BIN` for the first autonomous square not collected |
| 2026-11-17 | ARK Flow / rangefinder position offsets (FLOW_POS_*, RNGFND1_POS_*) not measured |
| 2026-11-23 | cuVSLAM at 22 Hz (normal 26–30) with skipped frames (66 ms between frames) → 0.3–0.5 s bridge holds, 8 in 16 min on the bench (camera itself 30 fps, 0 drops). Under the fast mission's 1.0 s VIO timeout, but watch it; check Orin load (`tegrastats`) and clocks |
| 2026-11-17 | Orin in 25 W mode with dynamic clocks; pinning clocks might steady cuVSLAM rate (untested) |
| 2026-11-18 | Bluetooth SSH written (`orin_setup/bluetooth_ssh/`), not installed or tested |
| 2026-11-08 | Orin has no RTC battery: clock reads 1969 without network |
| — | `packages/ros-camera/src/vio/CLAUDE.md` describes the OpenVINS era and is partly outdated; the wiki is current |
| — | Root `CLAUDE.md` / `AGENTS.md` describe upstream Synapse (ground robot), not the drone |
