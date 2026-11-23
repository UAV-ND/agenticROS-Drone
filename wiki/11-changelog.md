# 11 Changelog

[← wiki home](README.md)

Newest first. One line per change; PR numbers in brackets (merge dates in UTC). Add a line with every PR that changes behaviour.

## 2026-11

| Date | Change |
|---|---|
| 2026-11-23 | Rule added to `CLAUDE.md`, `AGENTS.md` and the wiki: every PR into `main` carries its wiki update; after merges, check every merged PR is in this changelog. [#17] |
| 2026-11-23 | Diagrams redrawn in Mermaid (README system + flight flow; wiki: navigation sources, boot services, VIO pipeline, bridge states, mission state machines, mission start sequence, development loop). [#16] |
| 2026-11-23 | Root `README.md` rewritten as the UAV-ND project overview (research, competition, codebase, wiki, credit to Corebrum Synapse); upstream README kept as `SYNAPSE_README.md`. [#14] |
| 2026-11-23 | Wiki created (`wiki/`). [#13] |
| 2026-11-23 | Fast 10 m square at 3 m: `path_mode: waypoints` (corner by corner, Cube plans each leg), `path_speed: 0.0` = Cube default speed (~3.8 m/s on 10 m sides), real-flight steps 1.0 → 2.0 → 0.0; `FcCommander.do_speed` (MAV_CMD_DO_CHANGE_SPEED); bridge jump gate 4 → 7 m/s; `select_mission.sh` / `~/.uav_mission` choose the boot mission. Decision: no Mission Planner parameter changes. [#13] |
| 2026-11-19/20 | **First autonomous 3 m square on VIO flown: complete success.** |
| 2026-11-19 | Autonomous 3 m VIO square started at boot by RC ch6 (`uav-mission` replaces `uav-fclink` at boot, `check_mission.sh`, per-boot log files); SITL kit in `uav_mission/sitl/`. [#12] |
| 2026-11-19 | Per-boot service logs (`boot####_*`); SSH over Bluetooth setup (not installed yet). [#11] |
| 2026-11-19 | Recordings numbered `run####` instead of dated (Orin clock reads 1969 outdoors). [#10] |
| 2026-11-19 | Teammates: dice detection YOLO model + tools in `packages/object-detection/` [#7]; drone-height die tracking in `die_tracking/` [#6, #8] (#6 merged by mistake, left as is). |
| 2026-11-18 | Outdoor flight on VIO, first LOITER on VIO (~0.2 m hold). |
| 2026-11-18 | `uav-fclink` boot service (read-only MAVROS + bridge + recorder, Cube-replug watchdog), `check_all.sh`. [#5] |
| 2026-11-18 | GPS-denied Cube settings file `cube_gps_denied.parm`, final D435i mount in the bridge, camera and cuVSLAM boot services with checks. [#4] |
| 2026-11-17 | Bench carry test on VIO passed. |
| 2026-11-13 | First real flights (manual, ARK Flow). |
| 2026-11-12 | MVP mission packages (`uav_vio`, `uav_gnc`, `uav_mission`), Cube parameter backup + checker, RC ch6 start [#1]; drone MCP tools `packages/synapse-drone-claude-code` [#2]. |
| 2026-11-09 | Airframe bench test: VIO fused on set 2 after restoring reverted parameters. |
| 2026-11-08 | Mission profiles hover / square / sweep-and-return; GPS-denied EKF origin in PREFLIGHT; mission tested in ArduCopter 4.6.3 SITL. |
| 2026-11-07 | Design decisions: ArduPilot + MAVROS (not PX4), switchable EKF sources, Jetson command scope, packages `uav_vio` / `uav_gnc` / `uav_mission`. |
| 2026-11-04 | cuVSLAM stereo-inertial live on the D435i (OpenVINS abandoned). |
