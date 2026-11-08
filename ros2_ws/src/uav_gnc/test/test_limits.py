import math

from uav_gnc.limits import Limits, clamp_hold, clamp_velocity

L = Limits(max_alt=5.0, min_alt=0.5, max_radius=20.0, max_speed=1.0, max_vz=0.5)
O = (10.0, -4.0, 1.0)       # origin: takeoff point, ground z = 1.0


def test_inside_limits_untouched():
    p, c = clamp_hold((12.0, -3.0, 4.0), O, L)
    assert p == (12.0, -3.0, 4.0) and not c


def test_altitude_clamped_relative_to_ground():
    assert clamp_hold((10.0, -4.0, 9.0), O, L) == ((10.0, -4.0, 6.0), True)
    assert clamp_hold((10.0, -4.0, 1.1), O, L) == ((10.0, -4.0, 1.5), True)


def test_radius_clamped_along_bearing():
    (x, y, z), c = clamp_hold((10.0 + 30.0, -4.0 + 40.0, 3.0), O, L)
    assert c and math.isclose(math.hypot(x - 10.0, y + 4.0), 20.0)
    assert math.isclose((x - 10.0) / (y + 4.0), 30.0 / 40.0)


def test_velocity_speed_caps():
    (vx, vy, vz), c = clamp_velocity((3.0, 4.0, 2.0), (10.0, -4.0, 4.0), O, L)
    assert c and math.isclose(math.hypot(vx, vy), 1.0) and vz == 0.5


def test_velocity_at_radius_only_outward_removed():
    pos = (10.0 + 20.0, -4.0, 4.0)                      # on the limit, east of origin
    (vx, vy, _), c = clamp_velocity((0.5, 0.3, 0.0), pos, O, L)
    assert c and math.isclose(vx, 0.0, abs_tol=1e-9) and math.isclose(vy, 0.3)
    (vx, _, _), _ = clamp_velocity((-0.5, 0.0, 0.0), pos, O, L)
    assert vx == -0.5                                   # moving back inside is allowed


def test_velocity_altitude_limits():
    assert clamp_velocity((0, 0, 0.3), (10.0, -4.0, 6.0), O, L)[0][2] == 0.0   # at max_alt
    assert clamp_velocity((0, 0, -0.3), (10.0, -4.0, 1.5), O, L)[0][2] == 0.0  # at min_alt
    assert clamp_velocity((0, 0, -0.3), (10.0, -4.0, 3.0), O, L)[0][2] == -0.3
