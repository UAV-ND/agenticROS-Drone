# UAV-ND Safety-Pilot Procedure (GN-4)

Use before every flight. The safety pilot holds the RC, maintains visual contact, and keeps immediate control. The agent may only request mission start/abort and read status; it never flies.

## No-go checks

- **No flight** until GN-2 verifies ch5 arm behavior and ch6 start/abort props-off; verify ch9's takeover position and mode on the actual radio map. Do not assume switch positions.
- **No flight** until N15 kill switch, Q3 battery failsafe, and Q7 RC-loss action are set and verified. This procedure is not flight authorization.
- Confirm the approved parameter file, profile, area, altitude, limits, battery threshold, and time limit. Do not use placeholder geometry. Clear the flight area, inspect the net, verify comms and battery levels.
- Confirm expected RC channels/mode. Ensure `rc_trigger` is running on the intended channel (`mission.launch.py` defaults it off). Check `/uav/rc_trigger/status` for fresh RC data and `ready_to_start: true`.
- Check fresh `/uav/fc_status` and `/uav/mission/state`; resolve every warning and `preflight_problems` item before starting.

## Start and monitor

1. Brief **ABORT** and **TAKEOVER** callouts. Keep throttle low for arming. Set ch6 LOW and verify `ready_to_start`, then move LOW to HIGH and hold for the default 0.5 s. This requests start; preflight may refuse. If refused, set LOW, fix the reported issue, then retry.
2. Verify expected FC mode, armed state, and takeoff. Once airborne and stable, center throttle as briefed. Keep eyes on the aircraft and hands on the RC; monitor mode, warnings, `/uav/mission/state`, and `/uav/fc_status`.

## Abort and takeover

- **Controlled mission landing:** while the mission is active and ch6 is HIGH, move HIGH to LOW and hold for the default 0.5 s. This requests `/uav/mission/abort`; airborne abort transitions to LAND. Verify the response and observe the landing. Stale RC/trigger data means the abort may not register.
- **Immediate danger or unacknowledged abort:** call **TAKEOVER**, move ch9 to its verified pilot-control position, confirm the FC mode, then fly and land manually. Mode change stops mission commands (`PILOT`); it does not land or disarm. Do not wait for mission state before taking control.
- Abort for an incursion, net/boundary concern, unexpected motion/altitude/mode, RC/FC or telemetry loss, VIO/FC warning, configured battery/time limit, or any uncertainty about control. Use approved configured thresholds only.
- After touchdown, keep clear until disarmed and propellers stop. Record final state, warnings, takeover/abort, and log locations. Never approach or disarm an airborne aircraft.