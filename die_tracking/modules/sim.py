#!/usr/bin/env python3
"""A simulated drone and downward camera for track_dice.py --sim, with no ArduPilot.

The drone answers the same MAVLink calls control.py makes (mode, arm, takeoff, velocity,
land), so the real controller, gains and signs are what fly it. The camera renders the
view straight down from the drone's simulated position over a floor with dice on it,
made from real Arducam images, so the real YOLO model has to find the die in it.

It is a test of the centering loop, not of flight: there is no wind, tilt or vibration,
only a delayed velocity response, a small position drift and a camera delay.
"""
import collections
import math
import os
import random
import time
from types import SimpleNamespace

import cv2
import numpy as np

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim_assets")
DICE = ("black_dice", "blue_dice", "yellow_dice", "red_dice")

PIXELS_PER_METER = 600  # resolution of the simulated floor
DIE_SIZE = 0.15  # m, the die's side
CROP_PADDING = 1.3  # the die images include 15% floor on each side
CAMERA_FOV = 65.0  # degrees across the image width; gives about the die size seen in training

RESPONSE_TIME = 0.5  # s for the drone to reach a commanded speed
CLIMB_SPEED = 0.5  # m/s, for takeoff and landing
GUID_TIMEOUT = 3.0  # s; like ArduPilot, stop if velocity commands stop arriving

# ArduPilot ids used by control.py
MODES = {"STABILIZE": 0, "GUIDED": 4, "LAND": 9}
CMD_ARM = 400
CMD_TAKEOFF = 22


def _floor_canvas():
    """Tile the floor image, mirrored so the seams match, into a 3x3 floor."""
    floor = cv2.imread(os.path.join(ASSETS, "floor.jpg"))
    row = np.hstack([cv2.flip(floor, 1), floor, cv2.flip(floor, 1)])
    return np.vstack([cv2.flip(row, 0), row, cv2.flip(row, 0)])


def _paste(canvas, image, center):
    """Blend a die image onto the floor, fading its square edges into the floor."""
    side = int(DIE_SIZE * CROP_PADDING * PIXELS_PER_METER)
    image = cv2.resize(image, (side, side), interpolation=cv2.INTER_AREA)
    mask = np.zeros((side, side), np.float32)
    cv2.circle(mask, (side // 2, side // 2), int(side * 0.42), 1.0, -1)
    mask = cv2.GaussianBlur(mask, (0, 0), side * 0.06)[..., None]
    x, y = int(center[0] - side / 2), int(center[1] - side / 2)
    region = canvas[y:y + side, x:x + side].astype(np.float32)
    canvas[y:y + side, x:x + side] = (image * mask + region * (1 - mask)).astype(np.uint8)


class Simulator:
    """A drone over a floor of dice. ``connection`` stands in for pymavlink's connection,
    and ``camera`` for cv2.VideoCapture."""

    def __init__(self, target, width=1920, height=1200, offset=(0.4, -0.5), drift=0.03,
                 latency=0.1, seed=None):
        self.random = random.Random(seed)
        self.width, self.height = width, height
        self.drift = drift
        self.latency = latency

        # Dice positions in meters, (north, east) from the takeoff point. The target
        # starts at ``offset``; the others are scattered around it.
        others = [(-0.45, 0.55), (0.85, 0.75), (-0.8, -0.9)]
        self.dice = {target: offset}
        for name, spot in zip([d for d in DICE if d != target], others):
            self.dice[name] = spot

        self.floor = _floor_canvas()
        self.origin = (self.floor.shape[1] / 2, self.floor.shape[0] / 2)
        for name, (north, east) in self.dice.items():
            _paste(self.floor, cv2.imread(os.path.join(ASSETS, name + ".png")), self._to_floor(north, east))
        self.target = target

        # Drone state; yaw is fixed at 0, so forward is north and right is east.
        self.north = self.east = self.altitude = 0.0
        self.velocity_north = self.velocity_east = 0.0
        self.drift_north = self.drift_east = 0.0
        self.command = (0.0, 0.0)
        self.command_time = 0.0
        self.target_altitude = 0.0
        self.armed = False
        self.mode = "STABILIZE"
        self.last_update = time.time()
        self.last_position_message = 0.0
        self.history = collections.deque(maxlen=200)
        self.trail = collections.deque(maxlen=2000)
        self.statustext = collections.deque()

        self.connection = _Connection(self)
        self.camera = _Camera(self)

    def _to_floor(self, north, east):
        return (self.origin[0] + east * PIXELS_PER_METER, self.origin[1] - north * PIXELS_PER_METER)

    def update(self):
        """Advance the simulated drone to the current time."""
        now = time.time()
        dt = min(now - self.last_update, 0.2)
        self.last_update = now
        if dt <= 0:
            return

        want_north = want_east = 0.0
        if self.mode == "GUIDED" and self.armed and self.altitude > 0.05 and now - self.command_time < GUID_TIMEOUT:
            want_north, want_east = self.command
        blend = min(dt / RESPONSE_TIME, 1.0)
        self.velocity_north += (want_north - self.velocity_north) * blend
        self.velocity_east += (want_east - self.velocity_east) * blend

        # Indoors the position estimate wanders, so the drone slowly drifts.
        if self.altitude > 0.05:
            for axis in ("drift_north", "drift_east"):
                value = getattr(self, axis) + self.random.gauss(0, self.drift) * dt
                setattr(self, axis, max(-self.drift, min(self.drift, value)))
            self.north += (self.velocity_north + self.drift_north) * dt
            self.east += (self.velocity_east + self.drift_east) * dt

        if self.mode == "LAND" and self.armed:
            self.altitude = max(0.0, self.altitude - CLIMB_SPEED * dt)
            if self.altitude == 0.0:
                self.armed = False
                self.statustext.append("Disarming motors")
        elif self.armed:
            step = max(-CLIMB_SPEED * dt, min(CLIMB_SPEED * dt, self.target_altitude - self.altitude))
            self.altitude += step

        self.history.append((now, self.north, self.east, self.altitude))
        self.trail.append((self.north, self.east))

    def state_at(self, when):
        """The drone's (north, east, altitude) at an earlier time, for the camera delay."""
        for moment, north, east, altitude in reversed(self.history):
            if moment <= when:
                return north, east, altitude
        return self.north, self.east, self.altitude

    def target_offset(self):
        """How far the drone is from straight above the target, in m (forward, right)."""
        north, east = self.dice[self.target]
        return north - self.north, east - self.east

    def render(self):
        """The downward camera image, as it looked ``latency`` seconds ago."""
        north, east, altitude = self.state_at(time.time() - self.latency)
        altitude = max(altitude, 0.25)  # the camera can't focus closer anyway
        view_width = 2 * altitude * math.tan(math.radians(CAMERA_FOV) / 2) * PIXELS_PER_METER
        scale = self.width / view_width
        center_x, center_y = self._to_floor(north, east)
        matrix = np.float32([
            [scale, 0, self.width / 2 - scale * center_x],
            [0, scale, self.height / 2 - scale * center_y],
        ])
        frame = cv2.warpAffine(self.floor, matrix, (self.width, self.height), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REFLECT)
        noise = np.random.normal(0, 3, frame.shape).astype(np.int16)
        return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    def draw_map(self, size=500, meters=4.0):
        """Top-down map: dice, the drone, its camera view and its path."""
        canvas = np.full((size, size, 3), 40, np.uint8)
        per_meter = size / meters

        def point(north, east):
            return (int(size / 2 + east * per_meter), int(size / 2 - north * per_meter))

        for line in range(-2, 3):
            cv2.line(canvas, point(line, -2), point(line, 2), (60, 60, 60), 1)
            cv2.line(canvas, point(-2, line), point(2, line), (60, 60, 60), 1)
        colors = {"black_dice": (90, 90, 90), "blue_dice": (255, 80, 0), "yellow_dice": (0, 220, 255), "red_dice": (0, 0, 255)}
        half = int(DIE_SIZE * per_meter / 2)
        for name, (north, east) in self.dice.items():
            x, y = point(north, east)
            cv2.rectangle(canvas, (x - half, y - half), (x + half, y + half), colors[name], -1)
            if name == self.target:
                cv2.rectangle(canvas, (x - half - 4, y - half - 4), (x + half + 4, y + half + 4), (255, 255, 255), 1)

        trail = [point(north, east) for north, east in self.trail]
        for start, end in zip(trail, trail[1:]):
            cv2.line(canvas, start, end, (0, 160, 0), 1)

        altitude = max(self.altitude, 0.25)
        half_width = altitude * math.tan(math.radians(CAMERA_FOV) / 2)
        half_height = half_width * self.height / self.width
        cv2.rectangle(canvas, point(self.north + half_height, self.east - half_width),
                      point(self.north - half_height, self.east + half_width), (0, 255, 0), 1)
        x, y = point(self.north, self.east)
        cv2.line(canvas, (x - 8, y), (x + 8, y), (255, 255, 255), 2)
        cv2.line(canvas, (x, y - 8), (x, y + 8), (255, 255, 255), 2)

        forward, right = self.target_offset()
        lines = [
            "{}  alt {:.2f} m  {}".format(self.mode, self.altitude, "armed" if self.armed else "disarmed"),
            "off target: {:.0f} cm (fwd {:+.0f}, right {:+.0f})".format(
                100 * math.hypot(forward, right), 100 * forward, 100 * right),
            "speed: {:.2f} m/s".format(math.hypot(self.velocity_north, self.velocity_east)),
        ]
        for row, text in enumerate(lines):
            cv2.putText(canvas, text, (10, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(canvas, "N", (size - 25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        return canvas


class _Mav:
    """The ``connection.mav`` calls control.py makes."""

    def __init__(self, sim):
        self.sim = sim

    def heartbeat_send(self, *args):
        pass

    def command_long_send(self, system, component, command, confirmation, *params):
        sim = self.sim
        sim.update()
        if command == CMD_ARM and params[0] == 1 and not sim.armed:
            if sim.mode == "GUIDED":
                sim.armed = True
                sim.statustext.append("Arming motors")
            else:
                sim.statustext.append("Arm: mode not armable")
        elif command == CMD_TAKEOFF:
            if sim.armed and sim.mode == "GUIDED" and sim.altitude < 0.2:
                sim.target_altitude = params[6]
            else:
                sim.statustext.append("Takeoff rejected")

    def set_position_target_local_ned_send(self, time_ms, system, component, frame, mask, x, y, z, forward, right, *rest):
        self.sim.update()
        self.sim.command = (forward, right)  # yaw is 0, so forward is north and right is east
        self.sim.command_time = time.time()


class _Connection:
    """Stands in for pymavlink's connection object in control.py."""

    target_system = 1
    target_component = 1

    def __init__(self, sim):
        self.sim = sim
        self.mav = _Mav(sim)
        self.messages = {"SYS_STATUS": SimpleNamespace(voltage_battery=12600)}

    @property
    def flightmode(self):
        return self.sim.mode

    def mode_mapping(self):
        return dict(MODES)

    def set_mode(self, mode_id):
        self.sim.update()
        self.sim.mode = {value: name for name, value in MODES.items()}[mode_id]
        if self.sim.mode == "GUIDED" and self.sim.armed:
            self.sim.target_altitude = self.sim.altitude

    def motors_armed(self):
        return self.sim.armed

    def recv_match(self, blocking=False):
        sim = self.sim
        sim.update()
        if sim.statustext:
            return SimpleNamespace(get_type=lambda: "STATUSTEXT", text=sim.statustext.popleft())
        if time.time() - sim.last_position_message >= 0.1:
            sim.last_position_message = time.time()
            return SimpleNamespace(get_type=lambda: "GLOBAL_POSITION_INT", relative_alt=int(sim.altitude * 1000))
        return None


class _Camera:
    """Stands in for cv2.VideoCapture; also shows the top-down map window."""

    def __init__(self, sim):
        self.sim = sim

    def isOpened(self):
        return True

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: self.sim.width, cv2.CAP_PROP_FRAME_HEIGHT: self.sim.height}.get(prop, 0)

    def read(self):
        self.sim.update()
        cv2.imshow("Sim map", self.sim.draw_map())
        return True, self.sim.render()

    def release(self):
        pass
