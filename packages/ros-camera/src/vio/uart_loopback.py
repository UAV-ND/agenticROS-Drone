#!/usr/bin/env python3
"""Orin UART loopback test: proves the Orin's own TX pin works.

  1. Unplug the Cube's TELEM2 connector from the Orin.
  2. Jumper J12 pin 8 (UART1_TX) to J12 pin 10 (UART1_RX).
  3. ~/mav_venv/bin/python uart_loopback.py            (default /dev/ttyTHS1)

PASS at every baud -> the Orin side is fine; the fault is the cable/connector or the Cube's RX.
Nothing received  -> the Orin's TX pin isn't driving (pinmux / wrong header pin / damaged).
"""
import os
import sys

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
ok = True
for baud in (57600, 115200, 921600):
    with serial.Serial(port, baud, timeout=0.5) as s:
        s.reset_input_buffer()
        data = os.urandom(256)
        s.write(data)
        s.flush()
        back = s.read(len(data))
        good = back == data
        ok &= good
        print(f'{baud:7d}: sent {len(data)} B, got {len(back)} B, '
              f'{"PASS" if good else "match " + str(sum(a == b for a, b in zip(data, back)))}')
print('PASS: Orin TX/RX work; check the cable to the Cube.' if ok else
      'FAIL: Orin TX not looping back (pin 8 not driving, or jumper not on pins 8-10).')
sys.exit(0 if ok else 1)
