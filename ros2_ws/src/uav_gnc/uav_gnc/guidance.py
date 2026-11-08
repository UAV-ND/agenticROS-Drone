"""Pure guidance math for the MVP mission (no ROS; unit-tested in test/test_guidance.py).

* sweep_path     lawnmower lanes over a rectangle
* PathFollower   moves a position target ("carrot") along a polyline at a set speed and never lets
                 it run more than `max_lead` ahead of the drone, so the FC's own cruise speed
                 (WPNAV_SPEED, 5 m/s by default) never comes into play
* CenterController  the centering capability ported from uavnd-aimm-2026 main_controller.py /
                 control.py: moving average of the target offset (MA_X / MA_Z deques there) feeding
                 a PID per axis. The old code drove yaw + forward speed from a forward camera and a
                 lidar; here the camera looks down, so both axes are body-frame horizontal
                 velocities (forward, left), converted to world ENU with the drone's yaw.
"""
import math
from collections import deque
from dataclasses import dataclass


def sweep_path(x_min, x_max, y_min, y_max, spacing, z):
    """Lanes parallel to x, stepping in y by `spacing`, alternating direction. Local ENU, metres."""
    if spacing <= 0 or x_max <= x_min or y_max < y_min:
        raise ValueError('bad search area / lane spacing')
    n = int(math.floor((y_max - y_min) / spacing + 1e-9)) + 1
    pts = []
    for i in range(n):
        y = y_min + i * spacing
        xs = (x_min, x_max) if i % 2 == 0 else (x_max, x_min)
        pts += [(xs[0], y, z), (xs[1], y, z)]
    return pts


class PathFollower:
    """Carrot along a polyline. `progress()` lets a sweep be resumed exactly where it left off."""

    def __init__(self, points, speed, max_lead=1.0, tol=0.3):
        if len(points) < 1:
            raise ValueError('empty path')
        self.pts = [tuple(map(float, p)) for p in points]
        self.speed, self.max_lead, self.tol = speed, max_lead, tol
        self.seg = 0                       # carrot is on segment pts[seg] -> pts[seg+1]
        self.carrot = self.pts[0]

    def _seg_len(self, i):
        a, b = self.pts[i], self.pts[i + 1]
        return math.dist(a, b)

    def at_end(self) -> bool:
        return self.seg >= len(self.pts) - 1

    def done(self, pos) -> bool:
        return self.at_end() and math.dist(pos, self.pts[-1]) <= self.tol

    def step(self, pos, dt):
        """Advance the carrot by speed*dt (if the drone keeps up) and return it."""
        # never let the carrot get more than max_lead ahead of the drone (it waits if the drone lags)
        budget = min(self.speed * max(dt, 0.0), self.max_lead - math.dist(pos, self.carrot))
        if budget <= 0:
            return self.carrot
        while budget > 0 and not self.at_end():
            b = self.pts[self.seg + 1]
            rem = math.dist(self.carrot, b)
            if budget >= rem:
                self.carrot, budget = b, budget - rem
                self.seg += 1
            else:
                f = budget / rem
                self.carrot = tuple(c + (e - c) * f for c, e in zip(self.carrot, b))
                budget = 0
        return self.carrot

    def progress(self):
        """(carrot, remaining points after it): enough to rebuild the rest of the path."""
        return self.carrot, self.pts[self.seg + 1:]


@dataclass
class PidGains:
    kp: float = 0.6        # (m/s) per m of offset
    ki: float = 0.05
    kd: float = 0.1
    i_max: float = 0.3     # m/s contribution cap
    out_max: float = 0.5   # m/s per axis


class Pid:
    def __init__(self, g: PidGains):
        self.g, self.i, self.prev = g, 0.0, None

    def reset(self):
        self.i, self.prev = 0.0, None

    def __call__(self, err, dt):
        g = self.g
        if dt > 0:
            self.i = max(-g.i_max, min(g.i_max, self.i + g.ki * err * dt))
        d = (err - self.prev) / dt if (self.prev is not None and dt > 0) else 0.0
        self.prev = err
        return max(-g.out_max, min(g.out_max, g.kp * err + self.i + g.kd * d))


class CenterController:
    """Drive the drone over the target. Offsets are body FLU metres (forward, left) of the target."""

    def __init__(self, gains: PidGains, ma_len=5):
        self.fwd, self.left = Pid(gains), Pid(gains)
        self.ma_f, self.ma_l = deque(maxlen=ma_len), deque(maxlen=ma_len)

    def reset(self):
        self.fwd.reset(), self.left.reset()
        self.ma_f.clear(), self.ma_l.clear()

    def add(self, fwd, left):
        self.ma_f.append(fwd)
        self.ma_l.append(left)

    def offset(self):
        """Moving-average offset (m), or None before the first sample."""
        if not self.ma_f:
            return None
        return sum(self.ma_f) / len(self.ma_f), sum(self.ma_l) / len(self.ma_l)

    def velocity_world(self, yaw, dt):
        """World ENU (vx, vy) to move toward the target; (0, 0) without data."""
        off = self.offset()
        if off is None:
            return 0.0, 0.0
        vf, vl = self.fwd(off[0], dt), self.left(off[1], dt)
        c, s = math.cos(yaw), math.sin(yaw)
        return c * vf - s * vl, s * vf + c * vl
