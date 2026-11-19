#!/usr/bin/env python3
"""Bluetooth network access point (NAP) for SSH over Bluetooth. Runs as uav-bt-nap.service (root).

Registers the Orin as a Bluetooth PAN "access point" with BlueZ. Each laptop that connects gets a
network interface that BlueZ adds to the bridge `pan0`, which has the fixed address 169.254.77.1
(set up by install_bt_ssh.sh). The laptop gives itself a link-local 169.254.x.x address, then:
    ssh uav-nd-orin@169.254.77.1
Only paired + trusted devices can connect (pair with bt_pair.py). Re-registers if bluetoothd restarts.
"""
import sys
import dbus
import dbus.mainloop.glib
from gi.repository import GLib

BRIDGE = 'pan0'
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()


def log(*a):
    print(*a, flush=True)


def register():
    try:
        mgr = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager')
        for path, ifaces in mgr.GetManagedObjects().items():
            if 'org.bluez.NetworkServer1' in ifaces:
                props = dbus.Interface(bus.get_object('org.bluez', path), 'org.freedesktop.DBus.Properties')
                props.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(True))
                srv = dbus.Interface(bus.get_object('org.bluez', path), 'org.bluez.NetworkServer1')
                try:
                    srv.Register('nap', BRIDGE)
                except dbus.DBusException as e:
                    if 'AlreadyExists' not in e.get_dbus_name():
                        raise
                name = props.Get('org.bluez.Adapter1', 'Alias')
                log(f'NAP registered on {path} ("{name}"), bridge {BRIDGE}: ssh uav-nd-orin@169.254.77.1')
                return True
        log('no Bluetooth adapter with NetworkServer1 yet')
    except dbus.DBusException as e:
        log(f'register failed: {e}')
    return False


def on_owner(new_owner):
    if new_owner:
        log('bluetoothd (re)started: registering')
        GLib.timeout_add_seconds(2, lambda: (register(), False)[1])
    else:
        log('bluetoothd gone; waiting')


bus.watch_name_owner('org.bluez', on_owner)


def retry():
    return not register()      # keep retrying every 5 s until it works once


GLib.timeout_add_seconds(5, retry)
try:
    GLib.MainLoop().run()
except KeyboardInterrupt:
    sys.exit(0)
