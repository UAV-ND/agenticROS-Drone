#!/bin/bash
# Set up SSH over Bluetooth on the Orin.   sudo bash install_bt_ssh.sh   [--uninstall]
#  1. NetworkManager bridge "pan0" with the fixed address 169.254.77.1/16 (link-local, no DHCP needed)
#  2. uav-bt-nap.service: registers the Orin as a Bluetooth network access point on pan0
# Then pair each laptop once with:  sudo python3 bt_pair.py
set -e
D="$(dirname "$(readlink -f "$0")")"
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
if [ "$1" = "--uninstall" ]; then
    systemctl disable --now uav-bt-nap.service 2>/dev/null || true
    rm -f /etc/systemd/system/uav-bt-nap.service; systemctl daemon-reload
    nmcli connection delete bt-pan 2>/dev/null || true
    echo "removed"; exit 0
fi
if ! nmcli -t -f NAME connection show | grep -qx bt-pan; then
    nmcli connection add type bridge ifname pan0 con-name bt-pan autoconnect yes \
        bridge.stp no ipv4.method manual ipv4.addresses 169.254.77.1/16 ipv6.method link-local
fi
nmcli connection up bt-pan
install -m 644 "$D/uav-bt-nap.service" /etc/systemd/system/uav-bt-nap.service
systemctl daemon-reload
systemctl enable --now uav-bt-nap.service
sleep 3
ip -4 -br addr show pan0
journalctl -u uav-bt-nap -n 5 --no-pager
echo
echo "Done. Pair a laptop once:  sudo python3 $D/bt_pair.py"
