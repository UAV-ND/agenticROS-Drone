# Dice Detection: User Manual

This manual takes you from nothing to running the dice detector live on the Arducam camera.
It takes about 15 minutes, most of it waiting for installs. No machine-learning knowledge needed.

The detector finds four kinds of dice in the camera image and draws a labeled box around each:
**black_dice, blue_dice, yellow_dice, red_dice**.

---

## 1. What you need

| | |
|---|---|
| **Computer** | macOS (Apple Silicon recommended), Windows 10/11, or Linux |
| **Python** | 3.10 or newer. Check with `python3 --version` (Windows: `python --version`) |
| **Git** | Check with `git --version` |
| **Camera** | Arducam 2.3MP AR0234 Color Global Shutter USB 3.0 module, plugged into a **USB 3** port (blue port, or marked SS). A built-in webcam also works for a quick test. |
| **GitHub access** | Your GitHub account must be a member of the **UAV-ND** organization |
| **Disk space** | About 3 GB (mostly PyTorch) |

Don't have Python? Install it from [python.org/downloads](https://www.python.org/downloads/).
On Windows, tick **"Add python.exe to PATH"** in the installer.

---

## 2. Download the code

Open a terminal (macOS: **Terminal** app; Windows: **PowerShell**) and run:

```bash
git clone -b CV https://github.com/UAV-ND/agenticROS-Drone.git
cd agenticROS-Drone/CV
```

If Git asks you to sign in, use your GitHub account. If you get *"Repository not found"*, you
don't have access to the UAV-ND organization yet; ask a team admin.

Already downloaded it earlier? Run `git checkout CV && git pull` inside `agenticROS-Drone` to get the
newest code and model.

**All commands below are run from inside the `CV` folder.**

---

## 3. Install the software

Create a private Python environment for this project, so it doesn't interfere with anything else
on your computer, then install the requirements.

**macOS / Linux**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Windows (PowerShell)**

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell says *"running scripts is disabled"*, run
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, then try again.

When it works, your prompt starts with `(.venv)`. **Every time you open a new terminal**, go to the
`CV` folder and run the `activate` line again before using the detector.

---

## 4. Connect the camera

1. Plug the Arducam into a USB 3 port.
2. Find its camera number:

   ```bash
   python detect.py --list-cameras
   ```

   Example output:

   ```
     camera 0: 1920x1080
     camera 1: 1920x1200  <- likely the Arducam
   ```

   The Arducam is the one at **1920x1200**. Note its number.
   (A few lines starting with `OpenCV:` or `WARN` after the list are normal.)

3. If the number isn't `0`, open `config.yaml` in any text editor and change `source:` under
   `camera:` to that number. Or add `--source <number>` to the commands below.

**macOS camera permission:** the first time, macOS asks whether your terminal app may use the
camera; click **Allow**. If you see `not authorized to capture video`, go to
**System Settings → Privacy & Security → Camera**, turn on your terminal app (Terminal, iTerm,
VS Code...), then **quit and reopen** the terminal.

---

## 5. Run the detector

```bash
python detect.py
```

The first run takes a few extra seconds to load. Check the startup line in the terminal: it should
say `1920x1200`. If it says something else, such as `1920x1080`, it is using a different camera, so
go back to step 4.

A window called **Dice detection** opens with the live camera image:

- Each die gets a box with its color name and a confidence score (0 to 1; higher means more sure).
- Top-left: the model version in use, how long each frame took (ms), and a count of each color
  in view.

**Keys** (click on the video window first, so it receives the key presses):

| Key | Action |
|---|---|
| `q` | Quit |
| `s` | Save the current camera frame (without boxes) to `captures/` |
| `c` | Start/stop saving a frame every second |

### Options

| Command | What it does |
|---|---|
| `python detect.py --source 1` | Use camera 1 instead of the one in `config.yaml` |
| `python detect.py --conf 0.3` | Show less certain detections too (default `0.5`). Raise it, e.g. `0.7`, if you see boxes on things that aren't dice |
| `python detect.py --weights models/dice_v1.pt` | Use a specific model version instead of the newest |
| `python detect.py --save-every 1` | Save a frame every second from the start, for collecting training images |

### Which model is used?

All trained versions are in `models/` (`dice_v1.pt`, `dice_v2.pt`, `dice_v3.pt`, ...). The detector
uses the highest number unless you pass `--weights`. `models/history.csv` lists each version's accuracy
and what data it was trained on; open it in Excel, Numbers or Google Sheets.

---

## 6. Troubleshooting

| Problem | Fix |
|---|---|
| `Could not open camera N` | Wrong camera number. Run `python detect.py --list-cameras` and use the number shown. |
| `not authorized to capture video` (macOS) | Give your terminal app camera permission (step 4), then quit and reopen the terminal. |
| `--list-cameras` doesn't show a 1920x1200 camera | Arducam not detected. Try another USB 3 port or cable, unplug and replug, and avoid USB hubs. |
| Window opens but the image is black or frozen | Another app (Zoom, Teams, Photo Booth, a second `detect.py`) is using the camera. Close it. |
| Very slow (over 200 ms per frame) | Normal on an older computer without a GPU. Apple Silicon Macs and NVIDIA GPUs are used automatically. |
| `ModuleNotFoundError: No module named 'ultralytics'` | The environment isn't active. Run the `activate` line from step 3. |
| `No models/dice_v*.pt found` | You are not in the `CV` folder, or `models/` is missing. `cd` into `agenticROS-Drone/CV`. |
| Black dice missed more than other colors | This is the current model's weakest color. Save those frames with `s` for the next model version. |
| Boxes on things that aren't dice | Raise the threshold: `--conf 0.7`. Save those frames with `s` so they can be added to training. |
| Dice missed or given the wrong color | Save those frames with `s`. They are the most useful images for the next model version. |
| Keys don't do anything | Click on the video window first; key presses go to whichever window is selected. |

---

## 7. Next steps: making the model better

The model improves by adding labeled images from this camera and retraining. That workflow
(capture → label in Roboflow → export → train) is in [`README.md`](README.md). It needs a Roboflow
API key from the team; ask whoever manages the Roboflow project.
