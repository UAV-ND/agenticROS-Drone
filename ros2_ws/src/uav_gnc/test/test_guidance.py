import math

import pytest

from uav_gnc.guidance import CenterController, PathFollower, Pid, PidGains, sweep_path


def test_sweep_path_lanes_alternate():
    pts = sweep_path(2, 6, 2, 4, 1.0, 3.0)
    assert pts == [(2, 2, 3.0), (6, 2, 3.0), (6, 3, 3.0), (2, 3, 3.0), (2, 4, 3.0), (6, 4, 3.0)]


def test_sweep_path_rejects_bad_area():
    with pytest.raises(ValueError):
        sweep_path(2, 2, 0, 1, 1.0, 3.0)
    with pytest.raises(ValueError):
        sweep_path(0, 1, 0, 1, 0.0, 3.0)


def test_follower_moves_at_speed_and_turns_corners():
    f = PathFollower([(0, 0, 3), (1, 0, 3), (1, 1, 3)], speed=0.5, max_lead=10)
    c = f.step((0, 0, 3), 1.0)
    assert c == pytest.approx((0.5, 0, 3))
    c = f.step(c, 1.0)
    assert c == pytest.approx((1, 0, 3))
    c = f.step(c, 1.0)
    assert c == pytest.approx((1, 0.5, 3))
    f.step(c, 5.0)
    assert f.at_end() and f.done((1, 1, 3)) and not f.done((1, 0.5, 3))


def test_follower_waits_for_lagging_drone():
    f = PathFollower([(0, 0, 3), (10, 0, 3)], speed=1.0, max_lead=1.0)
    c = f.step((0, 0, 3), 1.0)
    assert c == pytest.approx((1, 0, 3))
    assert f.step((0, 0, 3), 1.0) == pytest.approx((1.0, 0, 3))   # drone still at 0: carrot waits


def test_follower_progress_allows_exact_resume():
    pts = [(0, 0, 3), (2, 0, 3), (2, 1, 3)]
    f = PathFollower(pts, speed=1.0, max_lead=10)
    f.step((0, 0, 3), 1.5)
    carrot, rest = f.progress()
    assert carrot == pytest.approx((1.5, 0, 3)) and rest == [(2, 0, 3), (2, 1, 3)]


def test_pid_output_and_integral_capped():
    p = Pid(PidGains(kp=10, ki=10, kd=0, i_max=0.2, out_max=0.5))
    assert p(1.0, 0.1) == 0.5
    for _ in range(100):
        p(1.0, 0.1)
    assert p.i == 0.2


def test_center_controller_moving_average_and_world_rotation():
    cc = CenterController(PidGains(kp=1.0, ki=0, kd=0, out_max=5), ma_len=3)
    assert cc.velocity_world(0.0, 0.1) == (0.0, 0.0)
    for v in (1.0, 2.0, 3.0, 4.0):                  # window keeps the last 3
        cc.add(v, 0.0)
    assert cc.offset() == pytest.approx((3.0, 0.0))
    # target 3 m ahead; drone facing north (yaw 90 deg) -> move north
    vx, vy = cc.velocity_world(math.pi / 2, 0.1)
    assert vx == pytest.approx(0.0, abs=1e-9) and vy == pytest.approx(3.0)
