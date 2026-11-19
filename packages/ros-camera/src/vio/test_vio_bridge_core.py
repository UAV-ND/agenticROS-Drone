"""Unit tests for vio_bridge_core (no ROS needed):  python3 -m pytest -q test_vio_bridge_core.py"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

from vio_bridge_core import (Bridge, GateParams, Sample, R_LINK_FROM_OPTICAL, inv,
                             mount_transform, se3)

DT = 1 / 30
T_BC = mount_transform(0.12, 0.02, -0.05, 0.0, 30.0, 0.0)   # camera 12 cm fwd, 30 deg down


def truth_body(t):
    """Level flight: a 2 m-radius circle at 0.5 m/s with heading along the path, climbing slowly."""
    w = 0.25
    p = np.array([2 * np.sin(w * t), 2 - 2 * np.cos(w * t), 0.1 * t])
    R = Rot.from_euler('z', w * t).as_matrix()
    v_world = np.array([0.5 * np.cos(w * t), 0.5 * np.sin(w * t), 0.1])
    return se3(R, p), v_world, np.array([0.0, 0.0, w])


def camera_sample(t, T_wb, v_world, w_body, cov=1e-5, T_wo_true=T_BC):
    """What cuVSLAM would publish if the world were the body's start pose (level)."""
    T_oc = inv(T_wo_true) @ T_wb @ T_BC
    R_cb = T_BC[:3, :3].T
    w_c = R_cb @ w_body
    v_b = T_wb[:3, :3].T @ v_world
    p_bc = T_BC[:3, 3]
    v_c = R_cb @ (v_b + np.cross(w_body, p_bc))       # velocity of the camera origin
    return Sample(t, T_oc, v_c, w_c, np.eye(6) * cov, np.eye(6) * cov)


def run(bridge, n, jump_at=None, jump=np.zeros(3), t0=0.0):
    outs = []
    for k in range(n):
        t = t0 + k * DT
        T, v, w = truth_body(t)
        s = camera_sample(t, T, v, w)
        if jump_at is not None and k >= jump_at:
            s.T_oc = s.T_oc.copy()
            s.T_oc[:3, 3] += jump                     # cuVSLAM's estimate jumps and stays offset
        outs.append((t, bridge.process(s)))
    return outs


def test_frames_and_twist_recover_body_truth():
    b = Bridge(T_BC, GateParams(max_age_s=0))
    outs = run(b, 300)
    pub = [(t, o) for t, o in outs if o is not None]
    assert len(pub) == 300 - GateParams().recover_frames + 1
    for t, o in pub:
        T, v_world, w = truth_body(t)
        assert np.allclose(o.p, T[:3, 3], atol=1e-9)
        assert Rot.from_quat(o.q_xyzw).inv() * Rot.from_matrix(T[:3, :3]) is not None
        assert (Rot.from_quat(o.q_xyzw).inv() * Rot.from_matrix(T[:3, :3])).magnitude() < 1e-9
        assert np.allclose(o.v_b, T[:3, :3].T @ v_world, atol=1e-9)
        assert np.allclose(o.w_b, w, atol=1e-9)


def test_world_is_level_with_tilted_camera():
    # Pure forward body motion must come out as pure +x, with zero z, despite the 30 deg tilt.
    b = Bridge(T_BC, GateParams(max_age_s=0, recover_frames=1))
    for k in range(10):
        T_wb = se3(np.eye(3), [0.1 * k, 0, 0])
        o = b.process(camera_sample(k * DT, T_wb, np.array([3.0, 0, 0]), np.zeros(3)))
    assert np.allclose(o.p, [0.9, 0, 0], atol=1e-9)
    assert np.allclose(o.v_b, [3.0, 0, 0], atol=1e-9)


def test_init_waits_for_low_covariance():
    b = Bridge(T_BC, GateParams(max_age_s=0, recover_frames=5))
    T, v, w = truth_body(0)
    for k in range(9):    # cuVSLAM publishes cov 1.0 while initializing
        assert b.process(camera_sample(k * DT, T, v * 0, w * 0, cov=1.0)) is None
    outs = [b.process(camera_sample((9 + k) * DT, T, v * 0, w * 0)) for k in range(5)]
    assert outs[:4] == [None] * 4 and outs[4] is not None


def test_stale_and_duplicate_stamps_are_dropped():
    b = Bridge(T_BC, GateParams(max_age_s=0, recover_frames=1))
    T, v, w = truth_body(0)
    assert b.process(camera_sample(1.0, T, v, w)) is not None
    assert b.process(camera_sample(1.0, T, v, w)) is None
    assert b.process(camera_sample(0.9, T, v, w)) is None
    assert b.stats.stale == 2
    # too old relative to the node clock
    b2 = Bridge(T_BC, GateParams(max_age_s=0.25, recover_frames=1))
    assert b2.process(camera_sample(1.0, T, v, w), now=1.3) is None
    assert b2.stats.too_old == 1


def test_jump_is_absorbed_and_output_resumes():
    g = GateParams(max_age_s=0)
    b = Bridge(T_BC, g)
    outs = run(b, 400, jump_at=200, jump=np.array([3.0, -1.0, 2.0]))
    assert b.stats.jumps == 1
    # output pauses for recover_frames after the jump
    gap = [o is None for t, o in outs[200:200 + g.recover_frames]]
    assert all(gap)
    assert outs[200 + g.recover_frames][1] is not None
    # published track stays continuous and ends close to truth (only the dead-reckoned step is lost)
    pub = [(t, o) for t, o in outs if o is not None]
    steps = [np.linalg.norm(b2.p - a2.p) for (_, a2), (_, b2) in zip(pub, pub[1:])]
    assert max(steps) < 0.5 * g.recover_frames * DT + 0.05
    t_end, o_end = pub[-1]
    assert np.linalg.norm(o_end.p - truth_body(t_end)[0][:3, 3]) < 0.005


def test_rotation_jump_is_rejected():
    b = Bridge(T_BC, GateParams(max_age_s=0))
    outs = []
    for k in range(100):
        t = k * DT
        T, v, w = truth_body(t)
        s = camera_sample(t, T, v, w)
        if k >= 60:
            s.T_oc = s.T_oc @ se3(Rot.from_euler('z', 40, degrees=True).as_matrix(), [0, 0, 0])
        outs.append(b.process(s))
    assert b.stats.jumps == 1 and 'rotation' in b.stats.events[0][1] + b.stats.events[0][2]


def test_high_covariance_suppresses_without_reanchor():
    g = GateParams(max_age_s=0)
    b = Bridge(T_BC, g)
    for k in range(200):
        t = k * DT
        T, v, w = truth_body(t)
        o = b.process(camera_sample(t, T, v, w, cov=0.5 if 100 <= k < 110 else 1e-5))
        if 100 <= k < 110 + g.recover_frames - 1:
            assert o is None
    assert b.stats.jumps == 0 and b.stats.high_cov == 10
    assert np.allclose(o.p, truth_body(199 * DT)[0][:3, 3], atol=1e-9)   # no offset introduced


def test_plausible_gap_holds_output_but_keeps_motion():
    b = Bridge(T_BC, GateParams(max_age_s=0))
    run(b, 50)
    T, v, w = truth_body(3.0)
    assert b.process(camera_sample(3.0, T, v, w)) is None   # 1.4 s after the last frame
    assert b.stats.gaps == 1 and b.stats.jumps == 0 and b.state == Bridge.HOLD
    outs = run(b, 30, t0=3.0 + DT)
    assert np.allclose(outs[-1][1].p, truth_body(outs[-1][0])[0][:3, 3], atol=1e-9)


def test_gap_with_implausible_motion_is_a_jump():
    b = Bridge(T_BC, GateParams(max_age_s=0))
    run(b, 50)
    T, v, w = truth_body(3.0)
    s = camera_sample(3.0, T, v, w)
    s.T_oc = s.T_oc.copy()
    s.T_oc[:3, 3] += [20.0, 0, 0]
    b.process(s)
    assert b.stats.gaps == 1 and b.stats.jumps == 1


def test_camera_imu_leveling_on_a_slope():
    # Drone sits on a 5 deg slope (pitched nose-up: negative pitch in FLU) when cuVSLAM starts. Without leveling the
    # world would be tilted 5 deg; with camera_imu leveling, flat-ground motion has no z.
    slope = Rot.from_euler('y', -5, degrees=True).as_matrix()
    T_true_wo = se3(slope, [0, 0, 0]) @ T_BC          # true level world vs cuVSLAM odom
    b = Bridge(T_BC, GateParams(max_age_s=0, recover_frames=1), level_mode='camera_imu',
               level_samples=50)
    # accel at rest = +g "up" in the camera frame, sent in the optical frame
    T_wb0 = se3(slope, [0, 0, 0])
    R_wc = (T_wb0 @ T_BC)[:3, :3]
    a_link = R_wc.T @ np.array([0, 0, 9.81])
    a_opt = R_LINK_FROM_OPTICAL.T @ a_link
    for _ in range(50):
        b.add_imu(a_opt, np.zeros(3), np.eye(3))
    assert b.state == Bridge.WAIT_INIT
    assert b.measured_tilt_deg == pytest.approx(25.0, abs=1e-6)   # 30 deg mount, nose 5 deg up
    for k in range(10):
        T_wb = se3(np.eye(3), [0.1 * k, 0, 0]) @ T_wb0 if k == 0 else se3(slope, [0.1 * k, 0, 0])
        T_oc = inv(T_true_wo) @ T_wb @ T_BC
        o = b.process(Sample(k * DT, T_oc, np.zeros(3), np.zeros(3), np.eye(6) * 1e-5,
                             np.eye(6) * 1e-5))
    assert abs(o.p[2]) < 1e-9 and o.p[0] == pytest.approx(0.9, abs=1e-9)
