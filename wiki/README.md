# UAV-ND drone wiki

The project wiki for the **UAV @ Notre Dame** drone, built for the **Raytheon Autonomous Vehicle Competition
(AVC) 2026–27**: GPS-denied flight with a Jetson Orin Nano companion computer and a Cube Orange flight
controller (ArduCopter 4.6.3).

It is written for teammates **and for their AI coding agents**: point your agent at this folder
(`wiki/README.md`) before it works on the drone code. Last updated **2026-11-22**.

> **Agents: read [Rules for agents](#rules-for-agents-and-humans) first.** Then read the page for your
> task. The rest of this repository (`packages/`, `docs/`, `CLAUDE.md`, `AGENTS.md`) is mostly the upstream
> **Corebrum Synapse** project for ground robots; the drone work is listed in [Repository map](02-repo-map.md).

## Where things stand (2026-11-22)

| Milestone | Status |
|---|---|
| First manual flights (no GPS, ARK Flow) | done 2026-11-13 |
| VIO (D435i + cuVSLAM) fused by the Cube, bench carry test | passed 2026-11-17 |
| Boot services: camera, cuVSLAM, Cube link, health checks | done 2026-11-17/18 |
| Outdoor flight on VIO incl. LOITER on VIO | done 2026-11-18, held ~0.2 m |
| **Autonomous 3 m square on VIO** (started by RC switch) | **flown 2026-11-19/20: complete success**, corners within 7 cm, landed 3 cm from takeoff |
| Fast autonomous 10 m square at 3 m (Cube default speed, ~3.8 m/s) | code + simulator done, PR #13; real flights next (1 → 2 m/s → default) |
| Competition mission (transit, grid sweep, centre on die, return) | state machine written and unit-tested; not flown |
| Dice detection (YOLO, 4 colours) | separate track, `packages/object-detection/`, PR #9 open |

## Pages

| Page | Read it when you... |
|---|---|
| [01 System overview](01-system-overview.md) | need the big picture: hardware, data flow, who commands what |
| [02 Repository map](02-repo-map.md) | look for code; which folders are the drone and which are upstream Synapse |
| [03 VIO pipeline](03-vio-pipeline.md) | work on the camera, cuVSLAM, or the VIO → Cube bridge |
| [04 Missions](04-missions.md) | work on autonomous flight: state machine, mission files, safety limits |
| [05 Boot services and checks](05-boot-services-and-checks.md) | start/stop/debug what runs on the Orin, find logs |
| [06 Simulation (SITL)](06-simulation.md) | test mission code without the drone (required before flying changes) |
| [07 Flight operations](07-flight-operations.md) | prepare a flight day: switches, roles, procedures, lessons |
| [Flight-day checklist](flight-day-checklist.md) | are at the field (full step-by-step sheet) |
| [08 Flight log](08-flight-log.md) | want results of past flights and bench tests |
| [09 Cube parameters](09-cube-parameters.md) | touch anything about the flight controller's settings |
| [10 Development process](10-development-process.md) | change code: branches, tests, PRs, how to update this wiki |
| [11 Changelog](11-changelog.md) | want to know what changed recently |
| [12 Open items and known issues](12-open-items.md) | look for work, or hit something odd |

## Rules for agents (and humans)

These are team decisions. Don't work around them; ask the software lead if one blocks you.

1. **The Jetson never writes flight-controller parameters.** Humans change Cube parameters in Mission
   Planner. Code on the Orin may only: set mode (GUIDED / LAND / RTL), arm, take off, send position or
   velocity setpoints, set a temporary GUIDED speed limit, and switch the EKF source set. See
   [09 Cube parameters](09-cube-parameters.md).
2. **The RC pilot can always take over** (flight-mode switch), and SD (ch6) up aborts a mission. Never add
   code that fights the pilot.
3. **All commands to the Cube go through `uav_gnc.FcCommander`**, which clamps them to a safety box
   (`max_alt`, `max_radius`, `max_speed`, `max_vz`). Don't publish MAVROS setpoints from anywhere else.
4. **AI agents (Synapse / MCP) only start or abort missions and read status.** They never fly the drone directly.
5. **Test in the simulator before the drone.** Mission or flight-code changes need passing unit tests and a
   SITL run ([06](06-simulation.md)), then a props-off bench test, then a flight.
6. **Don't run `sudo` or system-changing commands on the Orin on someone's behalf**: print the command and let
   the human run it. Don't restart `uav-mission` while the drone is armed.
7. **Ask before outward actions**: pushing, opening/merging PRs. Before merging, confirm the PR number, title
   and branch with `gh pr view <n>`. Other teams open PRs in this repo; numbers are not ours in sequence.
8. **No personal emails or secrets in repo files** (package maintainer is `uavnd@nd.edu`). API keys like
   `.roboflow_key` stay git-ignored.
9. **Never delete recorded data**: `~/bags/`, `~/flight_logs/`, `~/vio_runs/` on the Orin.
10. **GUI tools (RViz, rqt, Mission Planner) need the Orin's own monitor or a laptop**; don't launch GUIs over SSH.
11. **Keep this wiki current**: when you change behaviour, procedures or results, update the page and add a
    line to the [changelog](11-changelog.md) in the same PR. See [10](10-development-process.md#updating-the-wiki).

## People and roles

- Software lead / GPS-denied flight owner: Jonathan (UAV-ND software team).
- VIO track earlier work: Preston.
- Flight crew roles (pilot, Jetson operator, spotter, logger) are in [07 Flight operations](07-flight-operations.md).
- Other teammates' tracks in this repo: dice detection / die tracking (`packages/object-detection/`,
  `die_tracking/`, PR #9), mission-3 reroute helper (PR #3), drone height (merged PR #6).
