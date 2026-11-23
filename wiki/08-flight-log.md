# 08 Flight log

[← wiki home](README.md)

Newest first. Data lives on the Orin in `~/flight_logs/` (and Cube `.BIN` files collected from the SD card).
Positions are the Cube's EKF estimate (what the navigation source believed), not ground truth.

| Date | What | Result | Data |
|---|---|---|---|
| 2026-11-22 | Fast 10 m square at 3 m, **simulator** | Cube default speed: peak 3.78 m/s, corners ≤ 7 cm, landed 1 cm from start | [06](06-simulation.md) |
| 2026-11-19/20 | **First autonomous 3 m square on VIO** (real drone, SD start) | **Complete success.** 57 s flight; corners within 5–7 cm, median 2 cm from the outline, 0.30 m/s, height 1.35–1.55 m; hover 3 cm and **landed 3 cm from takeoff**; gentle landing, auto-disarm; VIO 29.4 Hz, no gaps, no bridge holds; EKF vs VIO median 2.3 cm. Procedure issues: throttle left low, mode switch on STABILIZE | `first_autonomous_square_run0004/` (README, `analyze_square.py`, `analysis.txt`); boot 13. Cube .BIN not collected |
| 2026-11-18 | Props-off bench test of the mission service | Passed: start, ch6 abort, mode takeover | checklist F1 |
| 2026-11-18 | **Outdoor flight on VIO**, first LOITER on VIO | LOITER held ~0.2 m (< 0.5 m over 14 s); EKF followed VIO ~4 cm; sub-second bridge holds at takeoff/landing (variance 0.024 vs gate 0.02); vibration fine (Z median 7); both landings tipped (arm switch flipped mid-bounce); LOITER sticks were ~70 µs off centre; heater blocked arming ~2.5 min | `2026-11-18_outdoor_vio_flight/` (README, csv, Cube log 00000004) |
| 2026-11-17 | Bench carry test on VIO after parameter restore + final camera mount | Passed: EKF vs VIO median 1 mm (p95 1.7 cm); 0.45 m push → +x 0.45 m, < 3 cm sideways | — |
| 2026-11-13 | **First real flights** (manual, no GPS, ARK Flow; STABILIZE → ALT_HOLD, LAND) | 3 short hops, ~2 min airborne; vibration 7–8 m/s² median, no clipping; hover throttle 16%, ~12.5 A; flow quality ~145; "ground mag anomaly, yaw re-aligned" after every takeoff | Cube logs 00000002/3.BIN |
| 2026-11-09 | Airframe bench test, props off | Set 2 fused VIO after restoring 5 silently reverted params (EKF vs bridge 0.3 cm) | `REPORT_2026-11-09_airframe_bench_test.md` |
| 2026-11-04 | cuVSLAM live, handheld | ~30 Hz, 17–21 ms latency, 3–5 cm loop error over 16 m | `~/vio_runs/` |

## Analysing a new flight

1. Copy the recording: `~/flight_logs/run####_*_mission/` (reindex if the battery was pulled: `ros2 bag reindex`).
2. `python3 ~/flight_logs/first_autonomous_square_run0004/analyze_square.py <bag>` gives the state timeline,
   corner distances, tracking error, VIO rate/gaps/variance, sticks and battery (it assumes the 3 m square;
   change the corner list for other patterns).
3. Cube `.BIN`: mode changes, vibration, EKF innovations, statustext. Orin time ≈ Cube time + a fixed offset
   (6.27 s on 2026-11-18); match them by the arming event.
4. Write a `README.md` in a new `~/flight_logs/<date>_<name>/` folder and add a row here.
