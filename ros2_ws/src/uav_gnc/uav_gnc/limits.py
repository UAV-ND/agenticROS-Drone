"""Pure safety-limit math for FcCommander (no ROS; unit-tested)."""
import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_alt: float = 5.0        # m above the ground height at arming
    min_alt: float = 0.5        # m; hold targets never lower than this (landing is a mode, not a target)
    max_radius: float = 20.0    # m horizontal from the takeoff point


def clamp_hold(p, origin, lim: Limits):
    """Clamp a local ENU hold target. Returns ((x, y, z), clamped?)."""
    x, y, z = p
    ox, oy, oz = origin
    clamped = False
    dx, dy = x - ox, y - oy
    r = math.hypot(dx, dy)
    if r > lim.max_radius:
        s = lim.max_radius / r
        x, y = ox + dx * s, oy + dy * s
        clamped = True
    zc = min(max(z, oz + lim.min_alt), oz + lim.max_alt)
    if zc != z:
        z, clamped = zc, True
    return (x, y, z), clamped
