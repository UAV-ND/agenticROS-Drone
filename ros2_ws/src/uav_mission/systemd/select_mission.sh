#!/bin/bash
# Choose which mission uav-mission.service flies when the pilot flips SD (ch6).
#   bash select_mission.sh                       # list the missions and show the current one
#   bash select_mission.sh square_10m_fast_vio   # choose one (then restart the service)
# The choice is stored in ~/.uav_mission and survives reboots.
D="$(dirname "$(readlink -f "$0")")"
CFG=$D/../config
cur=$( [ -s ~/.uav_mission ] && tr -d '[:space:]' < ~/.uav_mission || echo square_3m_vio )
if [ -z "$1" ]; then
    echo "current mission: $cur"
    echo "available:"
    for f in "$CFG"/*.yaml; do
        n=$(basename "$f" .yaml); [ "$n" = mission ] && continue
        printf '  %-22s %s\n' "$n" "$(grep -m1 -E '^# UAV-ND' "$f" | sed 's/^# UAV-ND *//' | cut -c1-90)"
    done
    exit 0
fi
[ -f "$CFG/$1.yaml" ] || { echo "no such mission: $1 (run without arguments to list them)"; exit 1; }
echo "$1" > ~/.uav_mission
echo "mission set to: $1"
grep -E '^\s*(square_side|path_speed|takeoff_alt):' "$CFG/$1.yaml" | sed 's/^ */  /'
echo "Now restart the service (props off, drone disarmed):  sudo systemctl restart uav-mission"
echo "and check:  bash $D/check_mission.sh"
