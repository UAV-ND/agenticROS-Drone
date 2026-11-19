#!/bin/bash
# Run the boot-service checks (read-only): camera, cuVSLAM, Cube link (+ mission if it is running).   check_all.sh
D="$(dirname "$(readlink -f "$0")")"
rc=0
checks="check_camera check_cuvslam check_fclink"
systemctl is-active --quiet uav-mission.service 2>/dev/null && checks="$checks check_mission"
for c in $checks; do
    echo "=== ${c#check_} ==="
    bash "$D/$c.sh" || rc=1
    echo
done
[ $rc = 0 ] && echo "ALL READY" || echo "SOMETHING IS NOT READY (see FAIL lines above)"
exit $rc
