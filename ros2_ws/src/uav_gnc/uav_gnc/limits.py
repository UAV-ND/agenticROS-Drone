"""Pure safety-limit math for FcCommander (no ROS; unit-tested)."""
import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_alt: float = 5.0        # m above the ground height at arming
    min_alt: float = 0.5        # m; hold targets never lower than this (landing is a mode, not a target)
    max_radius: float = 20.0    # m horizontal from the takeoff point
    max_speed: float = 1.0      # m/s horizontal, velocity setpoints
    max_vz: float = 0.5         # m/s vertical, velocity setpoints


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


def clamp_velocity(v, pos, origin, lim: Limits):
    """Clamp a world ENU velocity setpoint (vx, vy, vz). Returns ((vx, vy, vz), clamped?).

    Speed caps always apply. At the radius / altitude limits the component that would push further
    out is removed, so the drone can still move back inside.
    """
    vx, vy, vz = v
    x, y, z = pos
    ox, oy, oz = origin
    clamped = False
    sp = math.hypot(vx, vy)
    if sp > lim.max_speed:
        vx, vy, clamped = vx * lim.max_speed / sp, vy * lim.max_speed / sp, True
    dx, dy = x - ox, y - oy
    r = math.hypot(dx, dy)
    if r >= lim.max_radius:
        ux, uy = dx / r, dy / r
        out = vx * ux + vy * uy
        if out > 0:
            vx, vy, clamped = vx - out * ux, vy - out * uy, True
    if abs(vz) > lim.max_vz:
        vz, clamped = math.copysign(lim.max_vz, vz), True
    if (z >= oz + lim.max_alt and vz > 0) or (z <= oz + lim.min_alt and vz < 0):
        vz, clamped = 0.0, True
    return (vx, vy, vz), clamped
