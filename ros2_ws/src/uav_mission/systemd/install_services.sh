#!/bin/bash
# Install / remove the UAV-ND boot services.   sudo bash install_services.sh [--uninstall]
set -e
D="$(dirname "$(readlink -f "$0")")"
UNITS="uav-camera.service uav-cuvslam.service uav-mission.service"
if [ "$(id -u)" != 0 ]; then echo "run with sudo"; exit 1; fi
if [ "$1" = "--uninstall" ]; then
    systemctl disable --now $UNITS || true
    for u in $UNITS; do rm -f /etc/systemd/system/$u; done
    systemctl daemon-reload; echo "removed"; exit 0
fi
for u in $UNITS; do install -m 644 "$D/$u" /etc/systemd/system/$u; done
systemctl daemon-reload
systemctl enable $UNITS
echo "Installed and enabled (start at next boot). Start now:  sudo systemctl start $UNITS"
echo "Status:  systemctl status $UNITS     Logs:  journalctl -u uav-mission -f"
