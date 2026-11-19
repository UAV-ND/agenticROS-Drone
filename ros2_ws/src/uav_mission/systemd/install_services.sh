#!/bin/bash
# Install / remove the UAV-ND boot services.
#   sudo bash install_services.sh camera              # only the camera (first step)
#   sudo bash install_services.sh camera cuvslam      # several
#   sudo bash install_services.sh                     # camera cuvslam fclink (NOT mission)
#   sudo bash install_services.sh mission             # mission: name it explicitly. It uses the Cube port
#                                                       like fclink, so installing it disables fclink.
#   sudo bash install_services.sh fclink              # back to the read-only link (disables mission)
#   sudo bash install_services.sh --uninstall [names] # remove (the default three if no names)
set -e
D="$(dirname "$(readlink -f "$0")")"
if [ "$(id -u)" != 0 ]; then echo "run with sudo"; exit 1; fi
mode=install
if [ "$1" = "--uninstall" ]; then mode=uninstall; shift; fi
names="${*:-camera cuvslam fclink}"
UNITS=""
for n in $names; do
    [ -f "$D/uav-$n.service" ] || { echo "unknown service '$n' (choose from: camera cuvslam fclink mission)"; exit 1; }
    UNITS="$UNITS uav-$n.service"
done
if [ $mode = uninstall ]; then
    systemctl disable --now $UNITS || true
    for u in $UNITS; do rm -f /etc/systemd/system/$u; done
    systemctl daemon-reload; echo "removed:$UNITS"; exit 0
fi
for u in $UNITS; do install -m 644 "$D/$u" /etc/systemd/system/$u; done
systemctl daemon-reload
# mission and fclink both need the Cube's USB port: enabling one disables the other at boot.
case " $names " in
    *" mission "*) if [ -f /etc/systemd/system/uav-fclink.service ]; then
                       systemctl disable --now uav-fclink.service || true
                       echo "Disabled uav-fclink (the mission service replaces it)"; fi ;;
    *" fclink "*)  if [ -f /etc/systemd/system/uav-mission.service ]; then
                       systemctl disable --now uav-mission.service || true
                       echo "Disabled uav-mission (the read-only link replaces it)"; fi ;;
esac
systemctl enable $UNITS
echo "Installed and enabled (they start at every boot):$UNITS"
echo "Start now:  sudo systemctl start$UNITS"
echo "Status:     systemctl status$UNITS"
echo "Logs:       journalctl -u uav-camera -f"
