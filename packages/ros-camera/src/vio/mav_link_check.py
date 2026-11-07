#!/usr/bin/env python3
"""LISTEN-ONLY check of the Jetson <-> Cube Orange MAVLink link (sends nothing to the FC).

  ~/mav_venv/bin/python mav_link_check.py                 # /dev/ttyTHS1 @ 921600, 15 s
  ~/mav_venv/bin/python mav_link_check.py --port /dev/ttyTHS1 --baud 921600 --secs 30

Wiring (Orin Nano devkit 40-pin J12 <-> Cube TELEM2, JST-GH):
  Cube pin 2 TX  -> J12 pin 10 (UART1 RX)
  Cube pin 3 RX  -> J12 pin 8  (UART1 TX)
  Cube pin 6 GND -> J12 pin 6  (GND)
  Cube pin 1 +5V -> NOT CONNECTED
FC params (Mission Planner): SERIAL2_PROTOCOL 2, SERIAL2_BAUD 921, BRD_SER2_RTSCTS 0, then reboot.

ArduPilot sends HEARTBEAT on every MAVLink port without being asked, so a heartbeat is enough to
prove wiring + baud. Other message rates depend on the FC's SR2_* stream params.
Exit code: 0 = heartbeat from an autopilot seen, 1 = nothing / garbage, 2 = port problem.
Run with the MAVROS node stopped (only one process can own the port).
"""
import argparse
import collections
import os
import subprocess
import sys
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--secs', type=float, default=15.0)
    a = ap.parse_args()

    if not os.path.exists(a.port):
        print(f'FAIL: {a.port} does not exist'); return 2
    if not os.access(a.port, os.R_OK | os.W_OK):
        print(f'FAIL: no read/write access to {a.port} (need group dialout)'); return 2
    busy = subprocess.run(['fuser', a.port], capture_output=True, text=True).stdout.strip()
    if busy:
        print(f'FAIL: {a.port} is in use by PID(s) {busy} (MAVROS? nvgetty?)'); return 2

    m = mavutil.mavlink_connection(a.port, baud=a.baud, autoreconnect=False,
                                   source_system=255, source_component=0)
    print(f'Listening on {a.port} @ {a.baud} for {a.secs:.0f} s (sending nothing)...')
    counts = collections.Counter()
    hb = None
    bad = 0
    t_end = time.time() + a.secs
    while time.time() < t_end:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'BAD_DATA':
            bad += 1
            continue
        counts[(msg.get_srcSystem(), msg.get_srcComponent(), t)] += 1
        if t == 'HEARTBEAT' and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb is None:
            hb = msg
            print(f'  HEARTBEAT from sys {msg.get_srcSystem()} comp {msg.get_srcComponent()}: '
                  f'autopilot {mavutil.mavlink.enums["MAV_AUTOPILOT"][msg.autopilot].name}, '
                  f'type {mavutil.mavlink.enums["MAV_TYPE"][msg.type].name}, '
                  f'armed {bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}')
        if t == 'STATUSTEXT':
            print(f'  STATUSTEXT: {msg.text}')

    total = sum(counts.values())
    print(f'\n{total} messages, {bad} bad bytes/packets in {a.secs:.0f} s')
    for (sysid, comp, t), n in sorted(counts.items(), key=lambda kv: -kv[1])[:25]:
        print(f'  sys {sysid:3d} comp {comp:3d}  {t:28s} {n / a.secs:6.1f} Hz')
    if hb is not None:
        print('\nPASS: autopilot heartbeat received; wiring and baud are good.')
        return 0
    if bad:
        print('\nFAIL: bytes arrive but don\'t decode -> baud mismatch (check SERIAL2_BAUD = 921 and reboot) '
              'or SERIAL2_PROTOCOL is not MAVLink.')
    else:
        print('\nFAIL: nothing received -> swap TX/RX, check GND, FC powered, SERIAL2_PROTOCOL 2, '
              'and that the cable is on TELEM2.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
