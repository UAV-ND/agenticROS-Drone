# SSH into the Orin over Bluetooth

The Orin acts as a Bluetooth network access point. A paired laptop connects to it and gets a small private network,
and the Orin is always at the same address on that network:

```
ssh uav-nd-orin@169.254.77.1
```

Range is about 10 m and it's slow, but enough for a terminal (`uavcheck`, logs, `sudo poweroff`). Bluetooth uses
2.4 GHz, the same band as the FrSky RC link, so use it on the ground and don't rely on it in flight.

## One-time setup on the Orin
```bash
sudo bash ~/Documents/uav-nd/agenticROS-Drone/orin_setup/bluetooth_ssh/install_bt_ssh.sh
```
This creates the bridge `pan0` with the fixed address 169.254.77.1, and the boot service `uav-bt-nap`.
To remove it: `sudo bash install_bt_ssh.sh --uninstall`.

## Pair a laptop (once per laptop)
1. On the Orin: `sudo python3 ~/Documents/uav-nd/agenticROS-Drone/orin_setup/bluetooth_ssh/bt_pair.py`
   This opens a 120 s pairing window, and the Orin shows up as **jetson-orin**.
2. On the laptop: Bluetooth settings → Add device → **jetson-orin**. Check that the 6-digit code matches the one the Orin
   prints, then accept.
3. The device is now trusted and reconnects later without pairing again. Outside a pairing window the Orin is not
   visible and nobody new can pair.

To list paired devices: `python3 bt_pair.py --list`. To remove one: `sudo python3 bt_pair.py --remove AA:BB:CC:DD:EE:FF`.

## Connect and SSH
- **Windows:** Control Panel → *Devices and Printers* (or Settings → Bluetooth & devices → More devices and printer
  settings). Right-click **jetson-orin** → *Connect using* → *Access point*. Windows gives itself a 169.254.x.x
  address automatically; this can take up to about a minute the first time. Then `ssh uav-nd-orin@169.254.77.1`.
  If no address appears, set the "Bluetooth Network Connection" adapter to the static IP 169.254.77.2,
  mask 255.255.0.0.
- **Linux (NetworkManager):** after pairing, `nmcli device connect <Orin MAC>`, or open the Bluetooth settings and turn
  on *Network*. Then use the same ssh command. The Orin's Bluetooth MAC is F8:3D:C6:57:07:42.
- **macOS:** System Settings → Network → add *Bluetooth PAN* → connect to jetson-orin.

## Checks on the Orin
```bash
systemctl status uav-bt-nap            # should say "NAP registered ... bridge pan0"
ip -4 -br addr show pan0               # 169.254.77.1/16
python3 bt_pair.py --list              # paired devices, connected=True while a laptop is on
```

## Other fixed addresses
| Link | Address |
|---|---|
| USB-C cable (the flashing port) | `ssh uav-nd-orin@192.168.55.1` |
| Bluetooth (this) | `ssh uav-nd-orin@169.254.77.1` |
| Campus Wi-Fi (eduroam) | Assigned by the university; can change. Show it with `hostname -I`. |
