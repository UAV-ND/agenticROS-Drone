# 07 Flight operations

[← wiki home](README.md)

The full step-by-step sheet is **[flight-day-checklist.md](flight-day-checklist.md)** (print it). This page is
the summary and the reasons behind it.

## Transmitter switches

| Channel | Switch | Function |
|---|---|---|
| ch9 | flight-mode switch | 1 STABILIZE, 2 ALT_HOLD, 3 LOITER, 4 LAND, 5–6 STABILIZE |
| ch10 | **SG** | EKF source: **LOW = ARK Flow (set 1)**, **MID = VIO (set 2)**, **never HIGH** (no aiding) |
| ch5 | **SF** (left, lower) | arm / disarm (HIGH → LOW = motors stop, even in the air) |
| ch6 | **SD** | mission start: **down = start**, back up in flight = abort and land. Live whenever `uav-mission` runs, **even on the ground with props on** |
| ch3 | throttle | low to arm; **centred once airborne** |

## Crew

Pilot (final say), safety officer / spotter, Jetson operator (checks + live mission state), logger.

## Before an autonomous flight

- `check_all.sh` → ALL READY; `check_mission.sh` shows the right mission file and MISSION READY.
- Cube warm: in the cold the IMU heater blocks arming for ~2.5 min. Wait.
- Drone on the marked spot, camera facing the first side of the pattern, **not moved since cuVSLAM started**.
- ch10 MID, ch5 LOW, throttle low, ch6 up, **flight-mode switch on LOITER** (so taking over is a *change* of the switch).
- Clear area: 3 m square → 3 × 3 m + 1.5 m margin; 10 m square → ~16 × 16 m, nothing taller than 2 m inside.
- Jetson operator runs `ros2 topic echo /uav/mission/state --field data`.

## During

- Flip SD down. **As soon as it climbs, throttle to centre** and hands off.
- Take over with the **flight-mode switch to ALT_HOLD**; SD up = land where it is; ch5 disarm only as a last resort.
- Take over if it leaves the marked area, goes too high, wobbles, or anyone is unsure. At 3–4 m/s decide early.

## After

- Don't touch ch5 while it lands; it disarms itself. Then SD up, ch5 LOW.
- `sudo poweroff` the Orin **before** pulling the battery.
- Collect the Cube `.BIN` (SD card `APM/LOGS/`, date reads 1969/1980) and the newest `run####_*_mission` and
  `boot####_*` folders for analysis.

## Lessons (why the checklist says what it says)

| Lesson | Source |
|---|---|
| Throttle must be low to arm, even in GUIDED | SITL 2026-11-08 |
| Taking over in LOITER/ALT_HOLD with throttle low = fast descent into the ground (2 SITL crashes) | SITL; real flight 2026-11-19/20 had throttle left low and mode switch on STABILIZE (no harm, but takeover would have dropped it) |
| Flipping the arm switch during the landing bounce tipped the drone twice | outdoor flight 2026-11-18 |
| Stop in a steady hover before LAND; baro lags the rangefinder near the ground | 2026-11-18 .BIN |
| ALT_HOLD vs LOITER only looks different **hands off for 30–60 s**; any stick push moves LOITER's hold point | 2026-11-18 .BIN |
| SG HIGH = set 3 = "EKF3 stopped aiding" | 2026-11-18 |
| Arming in LOITER without GPS fails "waiting for home" until an EKF origin exists (the mission sets it) | 2026-11-18 |
| The battery failsafe is not set for 6S (BATT_LOW_VOLT 10.5): watch time and voltage, land by ~3.6 V/cell | parameter review |
| Cube parameters have silently reverted twice: check them every flight day | 2026-11-09, 2026-11-17 |
