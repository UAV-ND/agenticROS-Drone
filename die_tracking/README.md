# Die tracking

Centers the drone above a die so it can be picked up. The Arducam points straight down. Every frame, the YOLO model (`models/dice_v3.pt`) finds the target die and measures how far its center is from the image center. `modules/control.py` then sends ArduPilot a sideways and forward/back speed to close that gap. When the die stays inside the green circle for 0.5 s, the screen shows **ALIGNED**.

Three ways to run it, from safest to real:

| Mode | Drone | Camera | Use it to |
|---|---|---|---|
| `--sim` | simulated | simulated | test the centering loop and tune gains |
| `--dry-run` | pretend (nothing sent) | real Arducam | check the move directions by moving a die by hand |
| `--mode test` / `--mode flight` | ArduPilot SITL or real drone | real Arducam | fly |

## Files

| Path | What it is |
|---|---|
| `track_dice.py` | Main script: camera → detection → commands |
| `modules/control.py` | MAVLink commands for ArduPilot: GUIDED, arm, takeoff, velocity, land |
| `modules/sim.py` | Simulated drone and camera for `--sim` and `--dry-run` |
| `models/dice_v3.pt` | Dice detector (black, blue, yellow, red); trained in `UAV-ND/rtx-software` `dice_detect/` |
| `sim_assets/` | Floor and die images, cut from real Arducam frames, for the simulator |
| `requirements.txt` | Python packages |

## 1. Set up (once)

Python 3.10 or newer. On a Mac, conda's `base` environment works.

```bash
git clone -b dice-tracking-sim --single-branch https://github.com/UAV-ND/agenticROS-Drone.git
cd agenticROS-Drone/die_tracking
pip install -r requirements.txt
```

This installs Ultralytics (and with it PyTorch and OpenCV), pymavlink and pyserial. On a Mac the model runs on the GPU (MPS); on a Jetson with CUDA PyTorch, it uses CUDA.

Check that it works without any hardware:

```bash
python track_dice.py --sim --target blue_dice --control P --show-commands
```

The first run on a Mac asks for camera permission for the terminal; allow it, then run again.

## 2. Simulator (no drone, no camera)

```bash
python track_dice.py --sim --target blue_dice --control P --show-commands
```

Two windows open:

- **Dice tracker**: the simulated downward camera view, with the detections, the green circle and the speed being commanded.
- **Sim map**: a top-down map with the dice, the drone (white cross), the camera's view (green box), its path, and how far it really is from the target in cm.

The simulated drone takes off to 1.5 m and flies to the die. When it is aligned, the terminal prints how far off it really is, for example `ALIGNED 7.3 s after takeoff; really 5 cm off`. Press `q` in the Dice tracker window to land and quit.

The simulator takes the same commands ArduPilot would, so the real gains and direction signs in `control.py` are what fly it. It simulates a 0.5 s delay to reach a commanded speed, a slow drift and a 0.1 s camera delay. It does not simulate wind, tilt, vibration or motion blur.

| Option | Default | Meaning |
|---|---|---|
| `--target` | (required) | `black_dice`, `blue_dice`, `yellow_dice` or `red_dice` |
| `--sim-offset FORWARD RIGHT` | `0.4 -0.5` | where the die starts, in m from the drone |
| `--sim-drift` | `0.03` | m/s the drone may drift |
| `--sim-latency` | `0.1` | seconds of camera delay |

## 3. Dry run (real Arducam, nothing sent to a drone)

Use this to check that each direction is right for the way the camera will be mounted.

```bash
python track_dice.py --dry-run --source 0 --target blue_dice --control P --show-commands
```

1. Check that the first line says `Camera 0: 1920x1200`. That is the Arducam. If it says `1920x1080`, it is the laptop camera: press `q` and try `--source 1`.
2. Hold the camera facing down, about 1–1.5 m above the floor, with the top of the image toward the drone's front.
3. Move the die around under the camera and watch the terminal:

| Die position in the image | Command printed |
|---|---|
| right of center | `RIGHT` |
| left of center | `LEFT` |
| toward the top | `FORWARD` |
| toward the bottom | `BACK` |
| inside the green circle, or not seen | `STOP` |

If a direction is backwards, flip `RIGHT_SIGN` or `FORWARD_SIGN` at the top of `modules/control.py`. If the camera is turned 90°, the axes need swapping in code.

Reading a command line:

```
  -> SET_POSITION_TARGET_LOCAL_NED  FORWARD 0.17, LEFT 0.25 m/s   (die dx -588 px, dy -409 px)
```

`SET_POSITION_TARGET_LOCAL_NED` is the MAVLink message sent to ArduPilot, with the speed relative to the drone's front. `dx`/`dy` are how far the die is from the image center (here 588 px left and 409 px up). Speed is 0.001 m/s per pixel of error, capped at 0.3 m/s.

## 4. ArduPilot SITL (optional)

This checks the real MAVLink link against real ArduPilot software. SITL does not simulate the camera, so the die in the image does not move as the drone moves; use `--sim` for that.

Set up once on a Mac (Apple Silicon):

```bash
conda create -n ardupilot-sitl python=3.11 -y
conda activate ardupilot-sitl
pip install "empy==3.3.4" pexpect future pymavlink MAVProxy pyserial dronecan "setuptools<81" gnureadline
cd ~/Desktop/uavnd
git clone --depth 1 --branch Copter-4.6.3 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot && ./waf configure --board sitl && ./waf copter
```

`setuptools<81` is needed: newer versions remove `pkg_resources`, and the build then hangs.

Run, in two terminals:

```bash
conda activate ardupilot-sitl && cd ~/Desktop/uavnd/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter
```

```bash
conda activate base && cd agenticROS-Drone/die_tracking && python track_dice.py --target blue_dice --source 0 --control P --show-commands
```

`sim_vehicle.py` sends to UDP ports 14550 and 14551; `--mode test` (the default) connects to `127.0.0.1:14551`.

## 5. Real drone

The flight controller must run **ArduCopter** (check in Mission Planner or QGroundControl). The commands will not work with PX4.

```bash
python track_dice.py --mode flight --target blue_dice --source 0 --control P
```

`--mode flight` connects over USB at `/dev/ttyACM0`, shows no window, and records the annotated video to `debug/dice_tracker.avi` and the commands to `debug/dice_tracker_velocity.csv`. Ctrl-C lands.

Before the first flight:

- [ ] Directions checked with `--dry-run`, with the camera as mounted
- [ ] The drone holds position in GUIDED without GPS (optical flow or another position source). Without that, GUIDED is refused.
- [ ] A pilot holds the RC transmitter. Switching out of GUIDED stops all commands from this script.
- [ ] Start with `--control P` and the default `MAX_SPEED = 0.3` m/s
- [ ] The Jetson connects over USB. For TELEM2 serial (`/dev/ttyTHS1`), the code needs a port and baud option added.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--target` | | die color to center over |
| `--source` | `0` | camera number, or a video file |
| `--mode` | `test` | `test`: SITL at `127.0.0.1:14551` with a window; `flight`: `/dev/ttyACM0`, records video |
| `--control` | `PID` | `P` or `PID` (gains in `modules/control.py`) |
| `--altitude` | `1.5` | takeoff and tracking height, m |
| `--margin` | `60` | radius in pixels of the circle that counts as aligned |
| `--hold` | `0.5` | seconds the die must stay in the circle for ALIGNED |
| `--conf` | `0.5` | minimum detection confidence |
| `--show-commands` | off | print every command sent to the drone |
| `--bench` | off | detection only, no drone and no commands |

## Not done yet

- **ALIGNED triggers nothing.** Descending and gripping still need to be added.
- **Gains not tuned on a real drone.**
- **Camera-to-gripper offset not handled.** The image center is assumed to be where the gripper grabs.
- **The drone never yaws or changes height while tracking.**
- **If the die is lost, the drone stops and hovers.** It does not search.
