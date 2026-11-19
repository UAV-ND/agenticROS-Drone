#!/usr/bin/env python3
"""Drone control over MAVLink for ArduPilot, used by track_dice.py.

The camera points straight down, so pixel errors become sideways and forward/back
velocity in GUIDED mode. Yaw and altitude are held. Velocity commands stop on their
own if they stop arriving (ArduPilot GUID_TIMEOUT).
"""
import time

from pymavlink import mavutil

# Starting values, not tuned on a real drone. Start with a low MAX_SPEED.
MAX_SPEED = 0.3  # m/s
MIN_ALTITUDE = 0.3  # m, same limits as packages/synapse-drone-claude-code/src/safety.ts
MAX_ALTITUDE = 10.0  # m
GAINS = {  # (Kp, Ki, Kd), in m/s per pixel of error
    "P": (0.001, 0.0, 0.0),
    "PID": (0.001, 0.0001, 0.0002),
}
MAX_INTEGRAL_SPEED = 0.1  # m/s, the most the I term can add

# Camera mounting: image x is the drone's right, and the top of the image is its front.
# Flip a sign if the drone moves away from the die on that axis.
RIGHT_SIGN = 1
FORWARD_SIGN = -1

VELOCITY_ONLY_MASK = 0x0DC7  # ArduPilot SET_POSITION_TARGET_LOCAL_NED: use velocity only

connection = None
flight_altitude = None
altitude = None
gains = GAINS["PID"]
x_delta = 0.0
y_delta = 0.0
right_speed = 0.0
forward_speed = 0.0
debug_yaw = None
debug_velocity = None
_x_state = {"integral": 0.0, "last": None}
_y_state = {"integral": 0.0, "last": None}
_last_time = None
_last_heartbeat = 0.0
_warned_mode = False
show_commands = False  # print every command sent to the drone
_last_shown = (0.0, None)


def _show(text, kind=None):
    """Print a command when show_commands is on. Repeated velocity commands of the
    same kind print at most 5 times a second."""
    global _last_shown
    if not show_commands:
        return
    now = time.time()
    if kind is not None and kind == _last_shown[1] and now - _last_shown[0] < 0.2:
        return
    _last_shown = (now, kind)
    print("  -> {}".format(text))


def _pump():
    """Read waiting messages to update mode and altitude, and send our heartbeat."""
    global altitude, _last_heartbeat
    while True:
        message = connection.recv_match(blocking=False)
        if message is None:
            break
        kind = message.get_type()
        if kind == "GLOBAL_POSITION_INT":
            altitude = message.relative_alt / 1000.0
        elif kind == "LOCAL_POSITION_NED" and altitude is None:
            altitude = -message.z
        elif kind == "STATUSTEXT":
            print("Drone: {}".format(message.text))

    if time.time() - _last_heartbeat >= 1.0:
        connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        _last_heartbeat = time.time()


def _wait_for(condition, timeout, repeat=None):
    """Wait until ``condition()`` is true, calling ``repeat()`` about once a second."""
    deadline = time.time() + timeout
    last_repeat = 0.0
    while time.time() < deadline:
        _pump()
        if condition():
            return True
        if repeat is not None and time.time() - last_repeat >= 1.0:
            repeat()
            last_repeat = time.time()
        time.sleep(0.05)
    return False


def _set_mode(name, timeout=5.0):
    """Switch flight mode, returning whether the drone confirmed it."""
    mode_id = connection.mode_mapping()[name]
    _show("SET_MODE {}".format(name))
    return _wait_for(lambda: connection.flightmode == name, timeout, lambda: connection.set_mode(mode_id))


def _request_stream(message_id, rate_hz):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1000000 / rate_hz, 0, 0, 0, 0, 0,
    )


def _send_velocity(forward, right):
    """Send a velocity in the drone's own frame: forward and right in m/s, altitude held."""
    if forward == 0 and right == 0:
        _show("SET_POSITION_TARGET_LOCAL_NED  STOP (forward 0, right 0)", "stop")
    else:
        words = "{} {:.2f}, {} {:.2f} m/s".format(
            "FORWARD" if forward > 0 else "BACK", abs(forward), "RIGHT" if right > 0 else "LEFT", abs(right))
        _show("SET_POSITION_TARGET_LOCAL_NED  {}   (die dx {:+.0f} px, dy {:+.0f} px)".format(words, x_delta, y_delta), "move")
    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_ONLY_MASK,
        0, 0, 0, forward, right, 0, 0, 0, 0, 0, 0,
    )


def _pid(error, state, dt):
    """Return the PID speed for one axis, in m/s."""
    kp, ki, kd = gains
    derivative = 0.0
    if state["last"] is not None and dt > 0:
        derivative = (error - state["last"]) / dt
    state["last"] = error
    if ki > 0:
        limit = MAX_INTEGRAL_SPEED / ki
        state["integral"] = max(-limit, min(limit, state["integral"] + error * dt))
    return kp * error + ki * state["integral"] + kd * derivative


def _clamp_speed(forward, right):
    """Limit the combined speed to MAX_SPEED, keeping the direction."""
    speed = (forward ** 2 + right ** 2) ** 0.5
    if speed > MAX_SPEED:
        return forward * MAX_SPEED / speed, right * MAX_SPEED / speed
    return forward, right


def connect_drone(address):
    """Connect to the flight controller and wait for its heartbeat."""
    global connection
    print("Connecting to {}".format(address))
    connection = mavutil.mavlink_connection(address)
    if connection.wait_heartbeat(timeout=30) is None:
        raise RuntimeError("No heartbeat from {} after 30 s".format(address))
    _request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    _request_stream(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    _request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)


def configure_PID(mode):
    """Choose the controller: "P" or "PID"."""
    global gains
    gains = GAINS[mode]


def set_flight_altitude(meters):
    """Set the altitude to fly at, within MIN_ALTITUDE and MAX_ALTITUDE."""
    global flight_altitude
    if not MIN_ALTITUDE <= meters <= MAX_ALTITUDE:
        raise ValueError("Altitude {} m is outside {}-{} m".format(meters, MIN_ALTITUDE, MAX_ALTITUDE))
    flight_altitude = meters


def initialize_debug_logs(path):
    """Open the velocity log at ``<path>_velocity.csv``."""
    global debug_velocity
    debug_velocity = open(path + "_velocity.csv", "w")
    debug_velocity.write("time,x_delta,y_delta,forward,right\n")


def print_drone_report():
    """Print the drone's mode, armed state, altitude and battery."""
    _wait_for(lambda: "SYS_STATUS" in connection.messages, 2.0)
    status = connection.messages.get("SYS_STATUS")
    print("Mode: {}  Armed: {}".format(connection.flightmode, bool(connection.motors_armed())))
    print("Altitude: {} m".format("unknown" if altitude is None else round(altitude, 2)))
    print("Battery: {} V".format("unknown" if status is None else round(status.voltage_battery / 1000.0, 2)))


def arm_and_takeoff(meters):
    """Switch to GUIDED, arm, and climb to ``meters``. Lands and raises if takeoff fails."""
    set_flight_altitude(meters)
    if not _set_mode("GUIDED"):
        raise RuntimeError("Drone did not enter GUIDED. Without GPS it needs a working position estimate first.")

    def send_arm():
        _show("COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM (arm)")
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0,
        )

    if not _wait_for(lambda: connection.motors_armed(), 15.0, send_arm):
        raise RuntimeError("Drone did not arm. Check the pre-arm messages above.")

    def send_takeoff():
        # Only until it starts climbing; ArduPilot rejects a takeoff while already flying.
        if altitude is None or altitude < 0.2:
            _show("COMMAND_LONG MAV_CMD_NAV_TAKEOFF {} m".format(meters))
            connection.mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, meters,
            )

    reached = lambda: altitude is not None and altitude >= meters * 0.9
    if not _wait_for(reached, 30.0, send_takeoff):
        land()
        raise RuntimeError("Did not reach {} m within 30 s; landing".format(meters))
    print("Reached {:.2f} m".format(altitude))


def setXdelta(pixels):
    """Set the die's sideways error: positive when it is right of the image center."""
    global x_delta
    x_delta = pixels


def setYdelta(pixels):
    """Set the die's up/down error: positive when it is below the image center."""
    global y_delta
    y_delta = pixels


def setZDelta(pixels):
    """Kept for track_dice.py; altitude is held, so this has no effect."""


def control_drone():
    """Turn the current pixel errors into a velocity command and send it."""
    global right_speed, forward_speed, _last_time, _warned_mode
    _pump()
    if connection.flightmode != "GUIDED":
        # The pilot or a failsafe has taken over, so send nothing.
        if not _warned_mode:
            print("Mode is {}; not sending commands".format(connection.flightmode))
            _warned_mode = True
        right_speed = forward_speed = 0.0
        return
    _warned_mode = False

    now = time.time()
    dt = 0.0 if _last_time is None else now - _last_time
    _last_time = now
    forward = FORWARD_SIGN * _pid(y_delta, _y_state, dt)
    right = RIGHT_SIGN * _pid(x_delta, _x_state, dt)
    forward_speed, right_speed = _clamp_speed(forward, right)
    _send_velocity(forward_speed, right_speed)

    if debug_velocity is not None:
        debug_velocity.write("{:.3f},{},{},{:.3f},{:.3f}\n".format(now, x_delta, y_delta, forward_speed, right_speed))


def stop_drone():
    """Hold position and clear the controller's memory."""
    global x_delta, y_delta, right_speed, forward_speed, _last_time
    x_delta = y_delta = 0.0
    right_speed = forward_speed = 0.0
    _last_time = None
    for state in (_x_state, _y_state):
        state["integral"] = 0.0
        state["last"] = None
    _pump()
    if connection.flightmode == "GUIDED":
        _send_velocity(0, 0)


def getMovementYawAngle():
    """Kept for track_dice.py; this controller never yaws."""
    return 0.0


def getVelocity():
    """Return the last commanded ``(forward, right)`` speed in m/s."""
    return forward_speed, right_speed


def land():
    """Switch to LAND. Never raises, because it runs while the script is shutting down."""
    try:
        if _set_mode("LAND"):
            print("Landing")
        else:
            print("WARNING: drone did not confirm LAND. Take over manually.")
    except Exception as error:
        print("WARNING: could not command LAND ({}). Take over manually.".format(error))
