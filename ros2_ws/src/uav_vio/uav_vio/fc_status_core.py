"""Pure-Python logic for the VIO/flight-controller status monitor (no ROS imports, unit-tested).

Answers one question for the pilot / mission layer: "which EKF3 source is the Cube using, and is
the VIO healthy enough for that?"

EKF source set, two independent signals:
  * RC switch (RC10_OPTION 90, "EKF Pos Source"): ArduPilot aux-switch positions are
    low (< 1200 us) -> set 1, middle -> set 2, high (> 1800 us) -> set 3.
  * STATUSTEXT "Using EKF Source Set N", which the Cube prints when it actually switches.
    This also catches switches that did not come from the RC (e.g. MAV_CMD_SET_EKF_SOURCE_SET).
  * A MAV_CMD_SET_EKF_SOURCE_SET the FC accepted (on_source_command). ArduPilot 4.6 switches
    SILENTLY on that command (GCS_Common.cpp handle_command_set_ekf_source_set): only the RC switch
    path prints "Using EKF Source Set N". So whoever sends the command must report it.
The latest FC-side value (STATUSTEXT or accepted command) is authoritative; RC is shown next to it.
"""
import json
import re
from dataclasses import dataclass, field
from typing import Optional

AUX_LOW_MAX_US = 1200
AUX_HIGH_MIN_US = 1800
SOURCE_SET_VIO = 2

_SRC_RE = re.compile(r'Using EKF Source Set (\d)')


def aux_switch_position(pwm: int) -> Optional[int]:
    """ArduPilot 3-position aux switch: 0 low, 1 middle, 2 high; None for no signal."""
    if pwm is None or pwm <= 0:
        return None
    if pwm < AUX_LOW_MAX_US:
        return 0
    if pwm > AUX_HIGH_MIN_US:
        return 2
    return 1


def source_set_from_rc(pwm: int) -> Optional[int]:
    pos = aux_switch_position(pwm)
    return None if pos is None else pos + 1


def source_set_from_statustext(text: str) -> Optional[int]:
    m = _SRC_RE.search(text or '')
    return int(m.group(1)) if m else None


@dataclass
class FcStatus:
    """Latest known state. Times are seconds on the monitor's clock (None = never seen)."""
    connected: bool = False
    armed: bool = False
    mode: str = ''
    rc_source_set: Optional[int] = None
    fc_source_set: Optional[int] = None        # from STATUSTEXT
    bridge_state: Optional[str] = None
    bridge_published: Optional[int] = None
    t_state: Optional[float] = None
    t_rc: Optional[float] = None
    t_bridge: Optional[float] = None
    ext_nav_in_use: bool = False               # "is using external nav data" seen since last switch
    recent_text: list = field(default_factory=list)

    def on_statustext(self, text: str, keep: int = 5):
        s = source_set_from_statustext(text)
        if s is not None:
            self.fc_source_set = s
            self.ext_nav_in_use = False
        if 'using external nav data' in (text or ''):
            self.ext_nav_in_use = True
        self.recent_text = (self.recent_text + [text])[-keep:]

    def on_source_command(self, source_set: int):
        """The FC accepted MAV_CMD_SET_EKF_SOURCE_SET (it prints nothing in that case)."""
        self.fc_source_set = int(source_set)
        self.ext_nav_in_use = False

    def on_bridge_status(self, raw_json: str, now: float):
        try:
            d = json.loads(raw_json)
        except (ValueError, TypeError):
            return
        self.bridge_state = d.get('state')
        self.bridge_published = d.get('published')
        self.t_bridge = now

    def effective_source_set(self) -> Optional[int]:
        return self.fc_source_set if self.fc_source_set is not None else self.rc_source_set

    def warnings(self, now: float, stale_s: float = 3.0) -> list:
        """Conditions a pilot should hear about. Empty list = all good."""
        w = []
        if self.t_state is None or now - self.t_state > stale_s:
            w.append('no MAVROS state (FCU link down?)')
        elif not self.connected:
            w.append('MAVROS not connected to FCU')
        bridge_alive = self.t_bridge is not None and now - self.t_bridge <= stale_s
        bridge_ok = bridge_alive and self.bridge_state == 'TRACKING'
        if self.effective_source_set() == SOURCE_SET_VIO and not bridge_ok:
            w.append('EKF on source set 2 (VIO) but bridge not tracking '
                     f'(state {self.bridge_state if bridge_alive else "no status"}): switch to set 1')
        if (self.rc_source_set is not None and self.fc_source_set is not None
                and self.rc_source_set != self.fc_source_set):
            w.append(f'RC switch says set {self.rc_source_set} but FC reports set {self.fc_source_set}')
        return w

    def summary(self, now: float) -> dict:
        return {
            'connected': self.connected, 'armed': self.armed, 'mode': self.mode,
            'source_set': self.effective_source_set(), 'rc_source_set': self.rc_source_set,
            'fc_source_set': self.fc_source_set, 'ext_nav_in_use': self.ext_nav_in_use,
            'bridge_state': self.bridge_state, 'bridge_published': self.bridge_published,
            'warnings': self.warnings(now),
        }
