# Flight-day checklist: manual LOITER (ARK Flow + VIO), the autonomous 3 m square, and the fast 10 m square on VIO

UAV-ND, Raytheon AVC 2026–27. Written 2026-11-17 after the bench carry test passed.
Updated 2026-11-19: the **mission service (`uav-mission`) now starts at boot** instead of the read-only Cube
link, so the Orin *can* fly the drone when SD (ch6) is flipped down. Landing procedure, Cube warm-up,
ALT_HOLD-vs-LOITER test and the autonomous square (Part F) added after the Nov 18 outdoor flight.

**Plan for the day, in this order:**
1. **Session 1, manual (Parts B–E):** the pilot flies. ALT_HOLD vs LOITER comparison and LOITER hold, first on
   ARK Flow (SG/ch10 LOW), then on VIO (SG MID). **SD (ch6) stays UP the whole session.**
2. **Session 2, autonomous square (Part F):** only if VIO LOITER in session 1 held within about 0.5 m hands off.
   The pilot starts it with SD and can take over at any time.

**Status of the autonomous square:** unit tests pass; props-off bench test on the drone passed (2026-11-18);
simulator test passed (2026-11-18: corners within 7 cm, landed on the start, SD abort and mode-switch takeover
both worked). **Flown on the real drone (2026-11-19/20): complete success**, corners within 5–7 cm, landed 3 cm
from the takeoff point. Next: the fast 10 m square at 3 m (Part G), stepping the speed up 1 → 2 m/s → Cube default.

**Not part of today:** CV / die detection, the competition sweep, and AI-agent commands.

---

## Roles

| Role | Job |
|---|---|
| **Pilot** | Flies. Has the final say on everything. Lands whenever unsure. |
| **Safety officer / spotter** | Watches the drone and the area, calls "LAND" out loud, keeps people back. |
| **Jetson operator** | Runs the check scripts on the Orin and watches the live status. Calls out if VIO stops tracking. In session 2, watches the mission state. |
| **Logger** | Writes down times, flight numbers and what happened (sheet at the end). |

---

## Switches (check on the transmitter before every flight)

| Channel | Switch on the transmitter | What it does | Position for takeoff |
|---|---|---|---|
| **Flight-mode switch (ch9)** | | 1 STABILIZE, 2 ALT_HOLD, **3 LOITER**, **4 LAND**, 5–6 STABILIZE | ALT_HOLD or STABILIZE (manual); **LOITER** before an autonomous mission (Part F) |
| **ch10** | **SG** | EKF source: **LOW = set 1 (ARK Flow)**, **MID = set 2 (VIO)**. **Never HIGH** (set 3 has no position source: "EKF3 stopped aiding", happened by mistake on Nov 18) | **LOW** (manual ARK Flow test) / **MID** (VIO flight, autonomous mission) |
| **ch5** | **SF** (left, lower) | **Arm / disarm switch.** HIGH → LOW disarms = motors stop, even in the air | **LOW = disarm.** `check_fclink.sh` shows its position. |
| **ch6** | **SD** | **Mission start (Jetson):** UP = LOW, DOWN = HIGH. Flipping **down** starts the autonomous mission, **even on the ground with props on**; back up in flight = abort (it lands). Active because `uav-mission` runs at boot | **UP (LOW)** for all manual flights; down only to start Part F |
| Throttle | left stick | Must be **low to arm**, **centred before entering ALT_HOLD / LOITER** (otherwise the drone climbs or descends) | Low |

---

## Part A: Before going to the field

- [ ] Batteries charged. **6S 6000 mAh**, full is about 25.2 V (4.2 V per cell).
- [ ] Transmitter charged.
- [ ] Laptop with Mission Planner, and a way to reach the Orin (monitor and keyboard, or SSH).
- [ ] Spare props, prop tool, tape (to mark the takeoff spot), zip ties.
- [ ] This checklist printed or on a phone.

## Part B: At the field, props OFF

### B1. Airframe
- [ ] Props off. Frame, arms and motors tight. No loose wires near the props.
- [ ] **D435i** firmly mounted. Nothing (props, cables, landing gear) in front of the lenses. Lens clean.
- [ ] Camera USB cable in the **USB 3** port and strain-relieved.
- [ ] **ARK Flow** lens (bottom) clean and unobstructed.
- [ ] Cube USB cable to the Orin firmly in. On the Orin it should be port 1-2.4; **don't use port 1-2.3**, which is faulty.
- [ ] Orin powered from the drone's power system (not the wall).

### B2. Mark the takeoff spot
- [ ] Choose an open spot with a **textured floor** and good light. ARK Flow and VIO both need visible texture, so avoid shiny, blank floors.
- [ ] Mark it with tape, and mark which way the camera points.
- [ ] Put the drone on the spot, level, camera facing the marked direction, and **power it on there**.
  cuVSLAM starts by itself about 1–2 minutes after the Orin boots, and VIO takes "level" and "forward"
  from where the drone sits at that moment. **From now until takeoff, don't pick the drone up or turn it.**
- [ ] If the drone was powered on somewhere else (or moved since), restart VIO on the spot:
  `sudo systemctl restart uav-cuvslam`, wait about 1 minute, then run `check_cuvslam.sh` (B4).

### B3. Check the Cube's parameters
The checker needs the Cube's USB port, which the mission service holds. Stop the service, check, start it again:
```bash
sudo systemctl stop uav-mission
~/mav_venv/bin/python ~/Documents/uav-nd/agenticROS-Drone/packages/ros-camera/src/vio/check_cube_params.py
sudo systemctl start uav-mission
```
- [ ] It must end with **`RESULT: ALL CRITICAL PARAMETERS MATCH`**.
- If it doesn't: load `cube_gps_denied.parm` (instructions at the top of that file, in the repo at
  `packages/ros-camera/src/vio/` and a copy in `~/Downloads`), with `uav-mission` still stopped. It needs **two
  passes**, with an unplug and replug of the Cube in between. Then run the check again, then start `uav-mission`.
  **Never load an old full parameter backup.** That is how the VIO settings got wiped twice.

### B4. Check the Jetson stack (it started by itself at boot)
Three services start at boot: **camera → cuVSLAM → mission service** (MAVROS + VIO bridge + mission, waiting
for SD + recorder). Wait about 2 minutes after power-on, then on the Orin (any terminal, any folder):
```bash
bash ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/src/uav_mission/systemd/check_all.sh
```
It runs the four checks below and ends with `ALL READY`. (Each also runs on its own: `check_camera.sh`,
`check_cuvslam.sh`, `check_fclink.sh`, `check_mission.sh` in the same folder.)
- [ ] `RESULT: CAMERA READY`: USB 3, both IR cameras about 30 fps, IMU about 200 Hz, emitter off.
- [ ] `RESULT: VIO READY`: VIO 20–30 Hz, small pose variance, `position` near 0 (VIO started where the drone is).
- [ ] `RESULT: CUBE LINK READY`: connected, Cube streaming, RC seen, bridge **TRACKING**, VIO to Cube about 20–30 Hz.
  It also prints the mode, arming state, EKF source set, and the ch5 / ch6 / ch10 positions.
- [ ] `check_mission`: state **IDLE** and the RC trigger **"ready: switch LOW"** (SD up). With SG/ch10 LOW it
  says `MISSION NOT READY ... VIO required`; **that's expected in session 1** (the mission only starts on VIO).
- If a check fails, read that service's output: `journalctl -u uav-camera -n 40` (or `uav-cuvslam`, `uav-mission`),
  or the per-boot log files in the newest `~/flight_logs/boot####_*/` (`camera.log`, `cuvslam.log`, `mission.log`).
  Restart one with `sudo systemctl restart uav-<name>`. Restarting `uav-camera` also restarts cuVSLAM.
- [ ] If the camera shows **USB 2**: re-seat the cable in a USB 3 port, then `sudo systemctl restart uav-camera`.

**Manual-only day, or extra safety in session 1 (optional):** `sudo systemctl start uav-fclink` swaps to the
read-only link for this power-on (it stops `uav-mission` automatically): the Orin then cannot arm or fly the drone
at all, and SD does nothing. Back to the mission: `sudo systemctl start uav-mission` (or reboot).
`check_fclink.sh` shows which of the two is running.

### B5. Recording (automatic)
The mission service records VIO, Cube and mission data (no images) every time it starts, into a numbered folder
`~/flight_logs/run####_<date>_<time>_mission/bag/` (it keeps the newest 40; the date may read 1969 without
internet, the number is what counts). Nothing to start by hand.
- [ ] Note the newest folder: `ls -td ~/flight_logs/run* | head -1`

**Live status for the Jetson operator** (leave this running in a terminal during the flights; 1 line per second):
```bash
source ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/install/setup.bash
ros2 topic echo /uav/fc_status --field data
```
Watch `bridge_state` (must stay **TRACKING**), `source_set`, `ext_nav_in_use` and `warnings`.

### B6. Props-off checks with the transmitter on
- [ ] In Mission Planner (Messages tab) or MAVProxy, the Cube is in STABILIZE with no errors.
- [ ] Flip **ch10 to MID**: a message containing **"using external nav data"** (e.g. "EKF3 IMU0 is using external nav data") appears and `fc_status` shows
  `"source_set": 2, "ext_nav_in_use": true`. Then flip **ch10 back to LOW**: `"source_set": 1`.
- [ ] Move the arm switch (ch5) and confirm with `check_fclink.sh` that ch5 changes. Leave it on **disarm (LOW)**.
- [ ] **SD (ch6) is UP** (`check_fclink.sh`: `ch6 mission LOW`). Pilot and spotter both know: SD down = the Orin takes off.
- [ ] Flight-mode switch: each position shows the right mode in Mission Planner (STABILIZE / ALT_HOLD / LOITER / LAND).
- [ ] The drone has not been moved since cuVSLAM started. **If it was:** put it back on the spot, `sudo systemctl restart uav-cuvslam`, wait about 1 minute, and run `check_cuvslam.sh` and `check_fclink.sh` again.

## Part C: Props ON, final checks

- [ ] Props on, correct direction for each motor, tight.
- [ ] Battery connected and strapped. Voltage reads about **4.0–4.2 V per cell** (24–25.2 V).
- [ ] Area clear. Spotter in position. Everyone behind the pilot or behind the nets.
- [ ] **ch10 LOW** (ARK Flow), throttle low, flight mode **STABILIZE or ALT_HOLD**, **SD (ch6) UP**.
- [ ] Jetson operator: `bridge_state` is still **TRACKING** in the live status.
- [ ] **Cube warmed up:** at least **3 minutes since the battery was connected.** The Cube keeps its sensors heated
      and refuses to arm until they reach 40 °C ("PreArm: heater temp low" / "Arm: heater temp low"). Outdoors on
      Nov 18 that took about 2.5 minutes, longer in the cold. If arming is refused with that message, just wait.
- [ ] Logger: note the time.
- [ ] Pilot calls "**arming**".

## Part D: The flight (step by step)

Keep every flight **short (about 3–5 minutes)**. **The low-battery warning is not set up for this 6S pack**, so the
pilot or logger watches flight time and voltage, and lands by **about 3.6 V per cell** (21.6 V) under load.

**Step 1: takeoff and hover in ALT_HOLD (source set 1)**
1. Arm (ch5) in STABILIZE or ALT_HOLD, as on Nov 13.
2. Climb to about **1.5 m** and switch to (or stay in) **ALT_HOLD** with the throttle **centred**.
3. Hover for about 20 s. Check it's steady.
   - A short heading twitch just after takeoff is expected ("ground mag anomaly"). It was seen on every Nov 13 flight.

**Step 1b: ALT_HOLD vs LOITER comparison (do this once per source set: ch10 LOW, then ch10 MID)**

*Why:* on Nov 18 the team felt ALT_HOLD and LOITER "behaved the same". The log shows LOITER was working
(LOITER 1 held within 0.19 m typical vs 0.63 m in ALT_HOLD 1), but it didn't look different because the pilot
was correcting by hand in ALT_HOLD (sticks pushed ~40% of the time) and kept nudging the sticks in LOITER, and the
phases were short (9–15 s). Any stick push past the 20 µs deadzone moves LOITER's hold point, so LOITER then just
follows the pilot. The difference only shows **hands off, for long enough.**

1. Hover at about **1.5 m** in **ALT_HOLD**, throttle centred, over a marked spot (tape cross on the floor).
   Logger calls "**ALT_HOLD, hands off**" and starts a timer.
2. **ALT_HOLD, hands off roll/pitch/yaw for 15 s** (throttle stays centred). Spotter watches the drift.
   **Expected:** it slowly drifts away, often more than 1 m. If it drifts more than about 1.5 m or towards a net or
   person, the pilot takes over early; that's still a valid result. Logger notes the rough drift and direction.
3. Pilot flies it back over the spot, sticks centred, then switches to **LOITER**.
   Logger calls "**LOITER, hands off**" and starts a timer.
4. **LOITER, hands off for 30–60 s.** Really hands off: thumbs off the roll/pitch/yaw sticks (or resting
   without pushing). **Expected:** it stays within about 0.5 m of the spot.
5. Logger writes both drifts in the log sheet ("ALT_HOLD drift / LOITER drift"), and the Cube time or wall-clock
   time of each phase, so software can measure them from the `.BIN`.
- If LOITER drifts as much as ALT_HOLD with the sticks truly released, that is a real problem: switch back to ALT_HOLD,
  land, and send the log.

**Step 2: LOITER on ARK Flow (ch10 LOW)**
1. Sticks centred, then switch to **LOITER**.
2. **Hands fully off** the roll/pitch/yaw sticks for about 30–60 s (throttle stays centred). **Expected:** the drone
   holds its spot within about 0.5 m. On Nov 18 the sticks were still slightly pushed (up to 75 µs off centre),
   which slowly moved the hold point, so the hold couldn't be measured cleanly.
3. Small stick inputs: move about 1 m forward, stop, and check it holds. Same left and right.
- **If LOITER is refused** (the mode doesn't change or there's a message): stay in ALT_HOLD, land, and send the log.

**Step 3: switch to VIO (ch10 MID) while holding steady in LOITER**
1. Spotter and Jetson operator ready. The pilot calls "**switching to VIO**".
2. Flip **ch10 to MID**.
3. **Expected:** no jump. The drone keeps holding its spot. The message "using external nav data" appears.
4. Hands off for about 20–30 s, then small moves as in step 2.
5. Flip **ch10 back to LOW** (ARK Flow) and check it still holds.

**Step 4: land** (both Nov 18 landings tipped over, 54° and 26°, because of how they ended)
1. **First stop in a steady hover** (ALT_HOLD or LOITER), about 1.5–2 m up, not moving up or down.
   On Nov 18, LAND was selected while already dropping at 1.8 m/s and the drone hit the ground hard and bounced.
2. Switch to **LAND** and leave the throttle **centred**. It comes down at about 0.5 m/s. It may touch down a little
   faster than that, because near the ground the Cube's height estimate lags behind.
3. **Don't touch the arm switch until the drone is sitting still on the ground.** LAND turns the motors off by itself
   after touchdown. Flipping ch5 in the air or during a bounce cuts the motors and the drone drops and tips
   (it happened on both Nov 18 flights). After it has disarmed by itself, put ch5 to LOW (disarm).
4. Landing in ALT_HOLD instead: lower the throttle gently, and once on the ground hold the throttle **fully down**
   until the motors stop (the Cube disarms by itself after up to 10 s, `DISARM_DELAY`), then ch5 LOW.
- Logger: note the time.

### Abort rules: switch to ALT_HOLD immediately (or LAND) if:
- it drifts more than about **1 m** with the sticks centred, or keeps sliding one way
- it **circles** in a growing spiral (toilet-bowling, usually a heading/compass problem)
- it oscillates, twitches, or jumps when ch10 is switched
- the Jetson operator calls "**VIO lost**" (`bridge_state` not TRACKING) while on set 2
- the altitude bounces or creeps up or down
- the spotter or pilot is unsure for any reason

**The two quick recoveries:**
1. **Flight mode to ALT_HOLD.** The pilot has full control again, with no position source needed.
2. **ch10 to LOW.** Back to ARK Flow.

**Automatic Cube failsafes:**
- RC signal lost → the Cube lands.
- EKF unhealthy → the Cube switches to LAND. Switching the flight mode takes control back.

## Part E: After landing

- [ ] Disarm, props safe.
- [ ] **Before disconnecting the battery, shut the Orin down cleanly** so the recording is closed properly:
  `sudo poweroff`, and wait until the Orin is off. (Pulling the battery first leaves the last recording without
  its index; it can be repaired with `ros2 bag reindex <folder>/bag`, but avoid it.)
  To keep the Orin running between flights instead, close the recording with `sudo systemctl restart uav-mission`
  (or `uav-fclink` if you swapped to it). That starts a new recording folder.
- [ ] Disconnect the battery.
- [ ] **Get the Cube log:** take out the SD card (or use Mission Planner's DataFlash download) and copy
  `APM/LOGS/` to the Orin, e.g. into `~/Downloads/`. **The file date will read 1969/1980 (no GPS): that's normal.**
  The newest number is today's flight.
- [ ] Note the recording folder in `~/flight_logs/` (`run####_..._mission`, or `run####_..._fclink` after a swap),
  and the newest `boot####_*` folder (service log files).
- [ ] Send both to software for analysis (VIO vs ARK Flow in flight, vibration, LOITER hold accuracy).

---

## Part F: Autonomous 3 m square on VIO (mission service)

**What it does:** the pilot flips **ch6 LOW → HIGH**. The Orin puts the Cube in GUIDED, arms, takes off to
**1.5 m**, waits until stable (2 s), flies a **3 m square at 0.3 m/s**: 3 m **forward** (the way the camera faces),
3 m **left**, 3 m back, 3 m right to the start; hovers 5 s over the start, then **LAND** (the Cube lands and
disarms itself). About 70 s. All on VIO (EKF source set 2). Settings: `ros2_ws/src/uav_mission/config/square_3m_vio.yaml`
(`square_side: -3.0` flies it forward-**right** instead).

**The Orin refuses to start** unless: link OK, EKF has a position, battery ≥ 50%, VIO healthy **and** ch10 on
**MID** (set 2). If it refuses, nothing happens; fix the reason (`check_mission.sh` prints it), flip ch6 LOW, then HIGH again.

**Automatic safety:** a box of 5 m radius / 2.5 m height / 0.5 m/s around the takeoff point; VIO lost > 1.5 s →
switch to ARK Flow, hold 3 s, land; low battery or 150 s → RTL (3 m); FC link lost → hold, land.

**Go / no-go for session 2:** session 1 on VIO (SG MID) held LOITER within about 0.5 m hands off for 30 s,
no "VIO lost" calls, landing went cleanly, battery ≥ 50% (fresh pack preferred).

### F0. One-time setup (on the Orin): DONE 2026-11-18
`uav-mission` is installed and starts at every boot instead of `uav-fclink` (code merged to `main`, PR #12).
Only needed again after a reflash:
```bash
cd ~/Documents/uav-nd/agenticROS-Drone/ros2_ws && colcon build --symlink-install --packages-select uav_mission
sudo bash ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/src/uav_mission/systemd/install_services.sh mission
```
Back to manual flying only at every boot: `sudo bash .../install_services.sh fclink` (disables the mission).

### F1. Props-OFF bench test: PASSED 2026-11-18 (repeat after any change to the mission code or settings)
1. Props **off**. Drone on the bench, transmitter on, ch10 **MID**, ch5 **LOW**, throttle **low**, ch6 **LOW**.
2. `check_all.sh` → all READY including `RESULT: MISSION READY`.
3. Flip **ch6 HIGH**. **Expected:** mode GUIDED, it arms, motors spin up (it tries to take off and can't), after
   about 30 s "takeoff timeout" → LAND → it disarms by itself. Watch with `ros2 topic echo /uav/mission/state`.
4. Repeat, and this time flip **ch6 LOW** while the motors spin: the mission should go to LAND and disarm.
5. Repeat, and take over with the **flight-mode switch** (move it to another position): mission state becomes
   `PILOT`; disarm with ch5 (HIGH → LOW) or let it disarm with throttle low.
If any step does something else, **don't fly**; send `~/flight_logs/boot####_*/mission.log` and the `run####_*_mission` folder.

### F2. Before each autonomous flight
- [ ] Parts B and C done (Cube warm, VIO tracking on the takeoff spot, drone not moved since cuVSLAM started).
- [ ] **Clear area:** 3 m × 3 m forward-left of the drone, plus 1.5 m margin around it; nobody inside.
      Mark the 4 corners on the ground so the spotter can judge the path.
- [ ] Drone on the spot, **camera facing the square's first side** (forward).
- [ ] Transmitter: ch10 **MID** (VIO), ch5 **LOW**, throttle **LOW**, ch6 **LOW**, flight-mode switch on **LOITER**
      (so taking over is a *change* of the switch, see below).
- [ ] `check_mission.sh` → `RESULT: MISSION READY`.
- [ ] Jetson operator: live mission state running in a terminal (1 line per second):
  ```bash
  source ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/install/setup.bash
  ros2 topic echo /uav/mission/state --field data
  ```
  Expected sequence: `PREFLIGHT` → `TAKEOFF` → `PATH` (watch `path_point` 1/5 … 5/5) → `HOVER` → `LAND` → `DONE`.
  Call out `FAILSAFE`, `RTL` or `ABORTED` immediately.

### F3. The flight
1. Pilot calls "**starting mission**", flips **ch6 HIGH**. The drone arms and climbs to 1.5 m.
2. **As soon as it is climbing, move the throttle stick to the centre and keep it there.** GUIDED ignores the
   sticks, but if the pilot takes over in LOITER / ALT_HOLD, a low throttle stick means a fast descent into
   the ground (this caused two crashes in simulation).
3. Sticks otherwise hands off. Spotter watches the corners; logger notes the time.
4. It lands and disarms by itself back near the start. **Don't touch ch5** while it lands (see Step 4 of Part D).
5. After it has disarmed: ch6 **LOW**, ch5 **LOW**.

**Taking over (any time):**
- **Flight-mode switch to ALT_HOLD** (a different position from where it was): the mission stops at once and the
  pilot flies. ArduPilot only reacts when the switch *moves*, so it must not already be sitting on the mode you want.
- **ch6 LOW** = abort: the mission lands where it is.
- ch5 HIGH → LOW = disarm = **motors stop in the air** (last resort only).

**Take over if:** it heads somewhere other than the marked square, climbs above about 2.5 m, drifts more than
about 1 m off the path, oscillates, or anyone is unsure.

### F4. After
- [ ] Note in the log sheet: did it finish? How far from the start did it land?
- [ ] Send the Cube `.BIN` and the newest `~/flight_logs/run####_*_mission` folder for analysis.

## Part G: Fast autonomous 10 m square at 3 m on VIO (after Part F has worked)

**What it does:** same start, abort and take-over as Part F (SD down = start, SD up = abort and land), but it
takes off to **3 m** and flies a **10 m square**: 10 m forward, 10 m left, 10 m back, 10 m right. The Orin
sends the Cube one corner at a time, and the Cube speeds up and brakes on its own and **stops on every corner**.
Then it hovers 5 s over the start and lands. All on VIO. Settings:
`ros2_ws/src/uav_mission/config/square_10m_fast_vio.yaml`.

**Speed:** the final flight uses the **Cube's own default top speed**: `path_speed: 0.0` = the Orin sends no
speed limit (we don't change any parameters). On 10 m sides the Cube's normal acceleration limit means it peaks
at **about 3.8 m/s**. `path_speed` 1.0 or 2.0 = a lower limit for the first flights. Simulator (2026-11-22):
corners within 6 cm, at most 0.4 m past a corner, height held within 2 cm, the square takes about 17–19 s.
The simulator has no camera, so **VIO at speed is untested**. Step the speed up one flight at a time:

| Flight | `path_speed` | Expected top speed | Go on to the next only if |
|---|---|---|---|
| G-1 | 1.0 | 1 m/s | DONE, corners on the marks, no VIO warnings, landed within about 0.5 m of the start |
| G-2 | 2.0 | 2 m/s | the same, and the logs show no VIO gaps or bridge rejections (send the run folder first) |
| G-3 | 0.0 (Cube default) | about 3.8 m/s | final speed |

**Automatic safety:** a box of 16 m radius / 4 m height around the takeoff point; VIO lost > 1.0 s →
ARK Flow, hold, land; low battery (35%) or 150 s → RTL; it won't start below 60% battery.

### G1. Choosing the fast mission (on the Orin, props off, disarmed)
```bash
cd ~/Documents/uav-nd/agenticROS-Drone/ros2_ws/src/uav_mission/systemd
bash select_mission.sh square_10m_fast_vio     # remembered across reboots
sudo systemctl restart uav-mission
bash check_mission.sh                          # parameters: .../square_10m_fast_vio.yaml, MISSION READY
```
To change the speed between flights: edit `path_speed:` in `config/square_10m_fast_vio.yaml` (`1.0`, `2.0`,
then `0.0` for the Cube default; write `0.0`, not `0`), then
`sudo systemctl restart uav-mission` (disarmed) and check again. **Back to the 3 m square:**
`bash select_mission.sh square_3m_vio` then restart. Before the first fast flight, repeat the **F1 bench test**
(props off) with the fast mission selected, because the mission code changed.

### G2. Before each fast flight (in addition to F2)
- [ ] **Clear area: 10 m × 10 m forward-left of the drone plus at least 3 m margin all round (about 16 × 16 m)**,
      nobody inside, nothing taller than 2 m inside. Mark the 4 corners.
- [ ] Spotter stands **outside** the square, at the start corner's side.
- [ ] Fresh pack (≥ 60%). Camera faces the first side.
- [ ] `check_mission.sh` shows `square_10m_fast_vio` and `RESULT: MISSION READY`.

### G3. The flight
Same as F3 (throttle stick to the centre once it climbs, hands off, don't touch ch5 while it lands).
**Take over (flight-mode switch to ALT_HOLD) if:** it goes more than about 1.5 m past a corner, heads outside
the marked square, climbs above about 4 m, wobbles in the turns, or anyone is unsure. At 3–4 m/s it covers a
metre in a quarter second, so the pilot decides early. ch6 LOW = it lands where it is.

### G4. After
- [ ] Log sheet: the speed setting, did it finish, distance of the landing from the start.
- [ ] Send the Cube `.BIN` and the newest `run####_*_mission` folder **before raising the speed**.

## Log sheet

| Flight # | Time up / down | Battery V before / after | Steps done (1–4) | ALT_HOLD drift, 15 s hands off (set 1 / set 2) | LOITER drift, 30–60 s hands off (set 1 / set 2) | Switch to VIO OK? | Notes |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |

**Autonomous square (Part F)**

| Run # | Time start / down | Battery V before | Final state (DONE / PILOT / FAILSAFE / ...) | Finished the square? | Landed how far from the start? | Took over? Why? | Notes |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |
| 2 | | | | | | | |

**Fast 10 m square (Part G)**

| Run # | `path_speed` | Time start / down | Battery V before | Final state | Finished? | Landed how far from the start? | Took over? Why? | Notes |
|---|---|---|---|---|---|---|---|---|
| G-1 | 1.0 | | | | | | | |
| G-2 | 2.0 | | | | | | | |
| G-3 | 0.0 (default) | | | | | | | |

Cube log file number(s): ________   Recording folder(s) in ~/flight_logs (run####): ________   Boot folder (boot####): ________

---

## Reference

| Item | Value |
|---|---|
| Camera mount | 134 mm forward, 110 mm below the Cube centre, ~0.6° up (in `vio_bridge.yaml`) |
| Bench carry test, 2026-11-17 | a push toward the camera = +0.45 m forward; the Cube followed VIO to about 1 mm |
| Nov 13 flights | vibration 7–8 m/s² (good), no clipping, hover throttle 16%, hover current about 12.5 A |
| Hover endurance | about 20 min per pack at hover (estimate); keep test flights to 3–5 min |
| RTL height | 3 m (`RTL_ALT` 300) |
| Nov 18 outdoor flight (VIO) | LOITER held about 0.2 m typical; Cube followed VIO within 4 cm; vibration fine; both landings tipped (arm switch flipped too early) |
| Settings file | `cube_gps_denied.parm`; check with `check_cube_params.py` (stop `uav-mission` first) |
| Boot services | `uav-camera`, `uav-cuvslam`, `uav-mission` (or `uav-fclink`, read-only, after a swap). Status: `systemctl status uav-<name>`; output: `journalctl -u uav-<name> -f` |
| Check scripts | `check_all.sh` (runs `check_camera.sh`, `check_cuvslam.sh`, `check_fclink.sh`, `check_mission.sh`) in `ros2_ws/src/uav_mission/systemd/` |
| Mission settings | `ros2_ws/src/uav_mission/config/square_3m_vio.yaml` (3 m, 1.5 m high, 0.3 m/s, VIO required); fast: `square_10m_fast_vio.yaml` (10 m, 3 m high, Cube default speed, about 3.8 m/s). Current choice: `bash select_mission.sh` |
| Simulator test | `ros2_ws/src/uav_mission/sitl/README.md` (no drone needed) |
| Cube unplugged / power-cycled | the mission service notices and restarts the link by itself within about 10 s. If `check_fclink.sh` fails after a replug: `sudo systemctl restart uav-mission` |
| Reaching the Orin | USB-C cable: `ssh uav-nd-orin@192.168.55.1` (Windows laptop needs the fixed address 192.168.55.2 on that adapter); campus Wi-Fi: address from `hostname -I` |
