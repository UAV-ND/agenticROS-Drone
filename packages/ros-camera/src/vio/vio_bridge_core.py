"""UAV-ND VIO -> ArduPilot bridge: frame math and safety gating (no ROS dependency).

Input:  cuVSLAM odometry. Pose of the camera (`camera_link`, FLU) in cuVSLAM's `odom` frame,
        which is the camera's pose at cuVSLAM start (identity at the first frame). It is NOT
        gravity-aligned: the camera is pitched ~30 deg down. Twist is in the child frame
        (camera_link); this was checked against finite differences on recorded runs.
Output: body (`base_link`, FLU) pose in a gravity-aligned world frame (ENU-like, arbitrary yaw)
        plus body-frame twist. MAVROS's odometry plugin turns that into MAVLink ODOMETRY
        (LOCAL_FRD / BODY_FRD) using its own static `odom_ned` / `base_link_frd` TFs.

World frame (T_wo, fixed once):
  level_mode "none":       world = the BODY pose at cuVSLAM start, i.e. T_wo = T_bc. Correct if
                           the drone sits level when cuVSLAM starts (on the ground, before arming).
  level_mode "camera_imu": world z = measured gravity. Uses the D435i accel while the vehicle is
                           still, so a sloped floor or a wrong mount pitch doesn't tilt the world.

Gating. cuVSLAM reports tracking OK through occlusion jumps of up to 37 m (Nov 4 loop bag), so
the bridge cannot trust its status alone. Per frame:
  * stale / duplicate / out-of-order stamps are dropped (never re-sent);
  * a JUMP (implied speed, rotation rate or twist above the limits) is absorbed: the output is dead-reckoned across that step with the last good velocity and the
    raw->output offset is re-anchored, so the published track stays continuous. MAVROS does not
    set ODOMETRY.reset_counter, so a discontinuity would otherwise hit EKF3 as a huge innovation;
  * a GAP in the input (cuVSLAM sometimes stops for 0.3-1.3 s) pauses output; the motion across
    the gap is kept if it passes the speed/rotation limits, otherwise it is treated as a jump;
  * HIGH COVARIANCE (cuVSLAM's own uncertainty) suppresses output without re-anchoring;
  * after any bad frame, output stays off until `recover_frames` consecutive good frames.
While output is off, EKF3 times out the external-nav source and falls back to source set 1
(ARK Flow), which is the intended failure behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as Rot

# camera_imu_optical_frame (x right, y down, z forward) -> camera_link (x fwd, y left, z up)
R_LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])


def se3(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def inv(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    return se3(R.T, -R.T @ p)


def mount_transform(x, y, z, roll_deg, pitch_down_deg, yaw_deg) -> np.ndarray:
    """T_bc: camera_link pose in base_link (both FLU). pitch_down_deg > 0 tilts the lens down
    (REP-103: positive pitch about +y turns +x toward -z)."""
    R = Rot.from_euler('ZYX', [yaw_deg, pitch_down_deg, roll_deg], degrees=True).as_matrix()
    return se3(R, [x, y, z])


def rotation_aligning(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit direction a onto unit direction b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = np.linalg.norm(v)
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        return Rot.from_rotvec(np.pi * axis / np.linalg.norm(axis)).as_matrix()
    return Rot.from_rotvec(v / s * np.arctan2(s, c)).as_matrix()


@dataclass
class GateParams:
    max_speed: float = 4.0            # m/s, implied by consecutive poses AND by the twist
    step_margin: float = 0.05         # m added to max_speed*dt (pose noise, timing jitter)
    max_ang_rate_deg: float = 300.0   # deg/s
    ang_margin_deg: float = 3.0
    max_pos_var: float = 0.02         # m^2, max of cuVSLAM's x/y/z pose variances
    max_gap_s: float = 0.3            # input gap that counts as a jump
    recover_frames: int = 15          # consecutive good frames before output resumes (0.5 s @ 30 Hz)
    predict_max_s: float = 0.5        # dead-reckon a jump with the last velocity for at most this long
    max_age_s: float = 0.25           # drop input older than this vs the node clock (<= 0 disables)
    min_pos_var: float = 1e-4         # output covariance floors
    min_rot_var: float = 1e-4
    min_vel_var: float = 1e-3


@dataclass
class Sample:
    stamp: float               # s
    T_oc: np.ndarray           # 4x4 camera_link pose in cuVSLAM odom
    v_c: np.ndarray            # linear velocity, camera_link frame
    w_c: np.ndarray            # angular velocity, camera_link frame
    pose_cov: np.ndarray       # 6x6, cuVSLAM odom frame
    twist_cov: np.ndarray      # 6x6, camera_link frame


@dataclass
class Output:
    stamp: float
    p: np.ndarray              # body position, world
    q_xyzw: np.ndarray         # body orientation, world
    v_b: np.ndarray            # body linear velocity, body frame
    w_b: np.ndarray            # body angular velocity, body frame
    pose_cov: np.ndarray       # 6x6, world frame
    twist_cov: np.ndarray      # 6x6, body frame


@dataclass
class Stats:
    received: int = 0
    published: int = 0
    stale: int = 0
    too_old: int = 0
    jumps: int = 0
    gaps: int = 0
    high_cov: int = 0
    holds: int = 0
    last_reject: str = ''
    jump_absorbed_m: float = 0.0
    events: list = field(default_factory=list)   # (stamp, kind, detail)


class Bridge:
    """Feed Samples in stamp order via process(); returns an Output to publish, or None."""

    WAIT_LEVEL, WAIT_INIT, TRACKING, HOLD = 'WAIT_LEVEL', 'WAIT_INIT', 'TRACKING', 'HOLD'

    def __init__(self, T_bc: np.ndarray, gate: GateParams | None = None, level_mode: str = 'none',
                 level_samples: int = 200, level_max_accel_std: float = 0.15,
                 level_max_gyro: float = 0.05):
        if level_mode not in ('none', 'camera_imu'):
            raise ValueError(f'level_mode must be none|camera_imu, got {level_mode!r}')
        self.T_bc = T_bc
        self.T_cb = inv(T_bc)
        self.R_bc = T_bc[:3, :3]
        self.p_cb = self.T_cb[:3, 3]          # body origin in camera frame
        self.g = gate or GateParams()
        self.level_mode = level_mode
        self.level_n = level_samples
        self.level_max_accel_std = level_max_accel_std
        self.level_max_gyro = level_max_gyro
        self._acc: list[np.ndarray] = []
        self.T_wo = T_bc.copy() if level_mode == 'none' else None
        self.measured_tilt_deg: float | None = None
        self.state = self.WAIT_INIT if level_mode == 'none' else self.WAIT_LEVEL
        self.T_off = np.eye(4)
        self.prev_stamp: float | None = None
        self.prev_raw: np.ndarray | None = None
        self.prev_corr: np.ndarray | None = None
        self.v_world = np.zeros(3)
        self.w_body = np.zeros(3)
        self.hold_since: float | None = None
        self.good = 0
        self.stats = Stats()

    # ---------- leveling ----------
    def add_imu(self, accel_optical, gyro_optical, R_oc: np.ndarray | None) -> None:
        """D435i IMU sample (camera_imu_optical_frame; IMU and camera share orientation, R = I)
        plus the latest cuVSLAM camera orientation in odom. Only used while WAIT_LEVEL."""
        if self.state != self.WAIT_LEVEL or R_oc is None:
            return
        a_c = R_LINK_FROM_OPTICAL @ np.asarray(accel_optical, float)
        w = np.linalg.norm(gyro_optical)
        if w > self.level_max_gyro:
            self._acc.clear()            # moving: restart the still window
            return
        self._acc.append(R_oc @ a_c)     # specific force in odom (points up at rest)
        if len(self._acc) < self.level_n:
            return
        A = np.array(self._acc[-self.level_n:])
        if np.linalg.norm(A.std(axis=0)) > self.level_max_accel_std:
            self._acc = self._acc[-self.level_n // 2:]
            return
        up_odom = A.mean(axis=0)
        R_wo = rotation_aligning(up_odom, np.array([0.0, 0.0, 1.0]))
        # Keep the world's yaw = the body's initial heading: remove any yaw that the minimal
        # rotation introduced relative to the body frame at cuVSLAM start.
        R_wb0 = R_wo @ self.T_cb[:3, :3]
        yaw = np.arctan2(R_wb0[1, 0], R_wb0[0, 0])
        R_wo = Rot.from_euler('z', -yaw).as_matrix() @ R_wo
        # Tilt of the camera's forward axis below horizontal (positive = looking down), at the
        # moment of leveling. With the drone level this is the actual mount pitch.
        up_cam = R_oc.T @ up_odom / np.linalg.norm(up_odom)
        self.measured_tilt_deg = float(-np.degrees(np.arcsin(np.clip(up_cam[0], -1, 1))))
        p0 = R_wo @ (self.T_cb[:3, 3])   # body origin at cuVSLAM start -> world origin
        self.T_wo = se3(R_wo, -p0)
        self.state = self.WAIT_INIT
        self._acc.clear()

    # ---------- main path ----------
    def _event(self, stamp, kind, detail):
        self.stats.events.append((stamp, kind, detail))
        self.stats.last_reject = f'{kind}: {detail}'

    def process(self, s: Sample, now: float | None = None) -> Output | None:
        st = self.stats
        st.received += 1
        if self.prev_stamp is not None and s.stamp <= self.prev_stamp:
            st.stale += 1
            return None
        if now is not None and self.g.max_age_s > 0 and now - s.stamp > self.g.max_age_s:
            st.too_old += 1
            return None
        if self.T_wo is None:            # still leveling: track the raw pose but publish nothing
            self.prev_stamp = s.stamp
            return None

        T_raw = self.T_wo @ s.T_oc @ self.T_cb          # body pose in world (uncorrected)
        v_b = self.R_bc @ (s.v_c + np.cross(s.w_c, self.p_cb))
        w_b = self.R_bc @ s.w_c
        pos_var = float(max(s.pose_cov[0, 0], s.pose_cov[1, 1], s.pose_cov[2, 2]))
        g = self.g

        if self.state == self.WAIT_INIT:
            self.prev_stamp, self.prev_raw = s.stamp, T_raw
            if pos_var > g.max_pos_var:
                self.good = 0
                return None
            self.good += 1
            if self.good < g.recover_frames:
                return None
            self.T_off = np.eye(4)
            self.prev_corr = T_raw
            self.state = self.TRACKING
            self.v_world = T_raw[:3, :3] @ v_b
            self.w_body = w_b
            return self._output(s.stamp, T_raw, v_b, w_b, s)

        dt = s.stamp - self.prev_stamp
        dR = self.prev_raw[:3, :3].T @ T_raw[:3, :3]
        step = float(np.linalg.norm(T_raw[:3, 3] - self.prev_raw[:3, 3]))
        ang = float(np.degrees(Rot.from_matrix(dR).magnitude()))
        jump = None
        gap = dt > g.max_gap_s     # input dropout: hold output, but keep the motion if it's plausible
        if gap:
            st.gaps += 1
        if step > g.max_speed * dt + g.step_margin:
            jump = f'step {step:.3f} m in {dt*1e3:.0f} ms'
        elif ang > g.max_ang_rate_deg * dt + g.ang_margin_deg:
            jump = f'rotation {ang:.1f} deg in {dt*1e3:.0f} ms'
        elif np.linalg.norm(v_b) > g.max_speed:
            jump = f'twist {np.linalg.norm(v_b):.2f} m/s'

        if jump is not None:
            st.jumps += 1
            held = 0.0 if self.hold_since is None else s.stamp - self.hold_since
            v = self.v_world if held < g.predict_max_s else np.zeros(3)
            w = self.w_body if held < g.predict_max_s else np.zeros(3)
            h = min(dt, g.max_gap_s)
            T_pred = self.prev_corr.copy()
            T_pred[:3, 3] += v * h
            T_pred[:3, :3] = self.prev_corr[:3, :3] @ Rot.from_rotvec(w * h).as_matrix()
            st.jump_absorbed_m += float(np.linalg.norm((self.T_off @ T_raw)[:3, 3] - T_pred[:3, 3]))
            self.T_off = T_pred @ inv(T_raw)
            self._event(s.stamp, 'jump', jump)
            self._enter_hold(s.stamp)
            T_corr = T_pred
        else:
            T_corr = self.T_off @ T_raw
            if gap:
                self._event(s.stamp, 'gap', f'{dt*1e3:.0f} ms without input (motion plausible, kept)')
                self._enter_hold(s.stamp)
            elif pos_var > g.max_pos_var:
                st.high_cov += 1
                self._event(s.stamp, 'high_cov', f'pos var {pos_var:.3g} m^2')
                self._enter_hold(s.stamp)
            elif self.state == self.HOLD:
                self.good += 1
                if self.good >= g.recover_frames:
                    self.state = self.TRACKING
                    self.hold_since = None
            if pos_var <= g.max_pos_var:
                self.v_world = T_corr[:3, :3] @ v_b
                self.w_body = w_b

        self.prev_stamp, self.prev_raw, self.prev_corr = s.stamp, T_raw, T_corr
        if self.state != self.TRACKING:
            return None
        return self._output(s.stamp, T_corr, v_b, w_b, s)

    def _enter_hold(self, stamp):
        if self.state != self.HOLD:
            self.stats.holds += 1
            self.hold_since = stamp
        self.state = self.HOLD
        self.good = 0

    def _output(self, stamp, T_wb, v_b, w_b, s: Sample) -> Output:
        g = self.g
        # pose covariance: odom frame -> world frame (rotation of the world w.r.t. odom)
        R = (self.T_off @ self.T_wo)[:3, :3]
        B = np.zeros((6, 6))
        B[:3, :3] = R
        B[3:, 3:] = R
        P = B @ s.pose_cov @ B.T
        # twist covariance: camera frame -> body frame (the lever-arm term is ignored)
        C = np.zeros((6, 6))
        C[:3, :3] = self.R_bc
        C[3:, 3:] = self.R_bc
        V = C @ s.twist_cov @ C.T
        for i in range(3):
            P[i, i] = max(P[i, i], g.min_pos_var)
            P[i + 3, i + 3] = max(P[i + 3, i + 3], g.min_rot_var)
            V[i, i] = max(V[i, i], g.min_vel_var)
            V[i + 3, i + 3] = max(V[i + 3, i + 3], g.min_rot_var)
        self.stats.published += 1
        return Output(stamp, T_wb[:3, 3].copy(), Rot.from_matrix(T_wb[:3, :3]).as_quat(),
                      v_b, w_b, P, V)
