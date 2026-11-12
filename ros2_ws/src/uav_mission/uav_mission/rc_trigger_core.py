"""RC switch -> mission start / abort (pure logic, unit-tested in test/test_rc_trigger_core.py).

The pilot starts the mission by moving a switch (default ch6) from LOW to HIGH:
  * edge-triggered: a switch that is already HIGH (e.g. at boot) does nothing until it has been
    seen LOW first, and every start needs a fresh LOW -> HIGH
  * debounced: the position must hold for `hold_s`
  * HIGH -> LOW while the mission is flying -> abort (mission lands), if `abort_on_low`
  * no RC data (stale / receiver lost) -> no action
The mission's own preflight still decides whether a start is accepted; a refused start needs the
switch cycled LOW -> HIGH again. A flight-mode change on the RC always overrides the mission.
"""
from dataclasses import dataclass
from typing import Optional

LOW, MID, HIGH = 'LOW', 'MID', 'HIGH'
INACTIVE_STATES = {'IDLE', 'DONE', 'PILOT', 'ABORTED', ''}   # mission not flying


@dataclass
class TriggerParams:
    low_us: int = 1300       # PWM below this = LOW
    high_us: int = 1700      # PWM above this = HIGH
    hold_s: float = 0.5      # position must be stable this long
    rc_timeout_s: float = 1.0
    abort_on_low: bool = True


def position(pwm: Optional[int], p: TriggerParams) -> Optional[str]:
    if pwm is None or pwm <= 0:
        return None
    if pwm < p.low_us:
        return LOW
    if pwm > p.high_us:
        return HIGH
    return MID


class RcTrigger:
    def __init__(self, p: TriggerParams = TriggerParams()):
        self.p = p
        self.stable = None          # debounced position
        self.cand, self.t_cand = None, None
        self.ready = False          # seen LOW since the last start (or since boot)
        self.t_rc = None
        self.status = 'waiting for RC'

    def on_rc(self, pwm: Optional[int], now: float):
        pos = position(pwm, self.p)
        if pos is None:
            return
        self.t_rc = now
        if pos != self.cand:
            self.cand, self.t_cand = pos, now

    def step(self, now: float, mission_state: str) -> Optional[str]:
        """Returns 'start', 'abort' or None."""
        if self.t_rc is None or now - self.t_rc > self.p.rc_timeout_s:
            self.status = 'no RC data'
            self.cand = self.t_cand = None
            return None
        if self.cand is None or now - self.t_cand < self.p.hold_s:
            return None                                  # still debouncing
        if self.cand == self.stable:
            return None
        prev, self.stable = self.stable, self.cand
        flying = mission_state not in INACTIVE_STATES
        if self.stable == LOW:
            self.ready = True
            if prev == HIGH and flying and self.p.abort_on_low:
                self.status = 'switch LOW in flight: abort'
                return 'abort'
            self.status = 'ready: switch LOW (flip HIGH to start)'
            return None
        if self.stable == HIGH:
            if self.ready and not flying:
                self.ready = False
                self.status = 'switch HIGH: start requested'
                return 'start'
            self.status = ('switch HIGH but not armed for start (flip LOW first)' if not self.ready
                           else f'switch HIGH, mission already {mission_state}')
            return None
        self.status = 'switch in middle: no action'
        return None
