import math

from uav_gnc.limits import Limits, clamp_hold

L = Limits(max_alt=5.0, min_alt=0.5, max_radius=20.0)
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
