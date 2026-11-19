#!/usr/bin/env python3
"""Pair a laptop/phone with the Orin over Bluetooth (run with sudo, then pair from the laptop).

    sudo python3 bt_pair.py            # pairing window open for 120 s
    sudo python3 bt_pair.py --list     # list paired devices
    sudo python3 bt_pair.py --remove AA:BB:CC:DD:EE:FF

Makes the Orin visible as "jetson-orin" and pairable for the window only, accepts the pairing
(confirm the 6-digit code shown on the laptop matches the one printed here) and marks the
device trusted, so it can reconnect later without pairing again. Outside the window the Orin is
not discoverable and nobody new can pair.
"""
import argparse
import sys
import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

AGENT_PATH = '/uavnd/btagent'
ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=int, default=120)
ap.add_argument('--list', action='store_true')
ap.add_argument('--remove')
a = ap.parse_args()

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()
mgr = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager')
objs = mgr.GetManagedObjects()
adapter_path = next(p for p, i in objs.items() if 'org.bluez.Adapter1' in i)
adapter = dbus.Interface(bus.get_object('org.bluez', adapter_path), 'org.freedesktop.DBus.Properties')

if a.list:
    for p, i in objs.items():
        d = i.get('org.bluez.Device1')
        if d and d.get('Paired'):
            print(f"{d['Address']}  {d.get('Alias', '')}  trusted={bool(d.get('Trusted'))}  connected={bool(d.get('Connected'))}")
    sys.exit(0)
if a.remove:
    for p, i in objs.items():
        d = i.get('org.bluez.Device1')
        if d and d['Address'].upper() == a.remove.upper():
            dbus.Interface(bus.get_object('org.bluez', adapter_path), 'org.bluez.Adapter1').RemoveDevice(p)
            print('removed', a.remove); sys.exit(0)
    print('not found'); sys.exit(1)


def trust(path):
    dev = dbus.Interface(bus.get_object('org.bluez', path), 'org.freedesktop.DBus.Properties')
    dev.Set('org.bluez.Device1', 'Trusted', dbus.Boolean(True))
    print(f"paired and trusted: {dev.Get('org.bluez.Device1', 'Address')} {dev.Get('org.bluez.Device1', 'Alias')}", flush=True)


class Agent(dbus.service.Object):
    @dbus.service.method('org.bluez.Agent1', in_signature='ou', out_signature='')
    def RequestConfirmation(self, device, passkey):
        print(f'pairing code {passkey:06d}: check the laptop shows the same code and accept there', flush=True)
        trust(device)

    @dbus.service.method('org.bluez.Agent1', in_signature='o', out_signature='')
    def RequestAuthorization(self, device):
        trust(device)

    @dbus.service.method('org.bluez.Agent1', in_signature='os', out_signature='')
    def AuthorizeService(self, device, uuid):
        return

    @dbus.service.method('org.bluez.Agent1', in_signature='ou', out_signature='')
    def DisplayPasskey(self, device, passkey):
        print(f'passkey {passkey:06d}', flush=True)

    @dbus.service.method('org.bluez.Agent1', in_signature='', out_signature='')
    def Release(self):
        pass

    @dbus.service.method('org.bluez.Agent1', in_signature='', out_signature='')
    def Cancel(self):
        print('pairing cancelled', flush=True)


Agent(bus, AGENT_PATH)
am = dbus.Interface(bus.get_object('org.bluez', '/org/bluez'), 'org.bluez.AgentManager1')
am.RegisterAgent(AGENT_PATH, 'DisplayYesNo')
am.RequestDefaultAgent(AGENT_PATH)
adapter.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(True))
adapter.Set('org.bluez.Adapter1', 'PairableTimeout', dbus.UInt32(a.seconds))
adapter.Set('org.bluez.Adapter1', 'DiscoverableTimeout', dbus.UInt32(a.seconds))
adapter.Set('org.bluez.Adapter1', 'Pairable', dbus.Boolean(True))
adapter.Set('org.bluez.Adapter1', 'Discoverable', dbus.Boolean(True))
name = adapter.Get('org.bluez.Adapter1', 'Alias')
print(f'"{name}" is visible and pairable for {a.seconds} s. On the laptop: Bluetooth settings -> add device -> {name}', flush=True)


def done():
    adapter.Set('org.bluez.Adapter1', 'Discoverable', dbus.Boolean(False))
    adapter.Set('org.bluez.Adapter1', 'Pairable', dbus.Boolean(False))
    am.UnregisterAgent(AGENT_PATH)
    print('pairing window closed', flush=True)
    loop.quit()


loop = GLib.MainLoop()
GLib.timeout_add_seconds(a.seconds, done)
try:
    loop.run()
except KeyboardInterrupt:
    done()
