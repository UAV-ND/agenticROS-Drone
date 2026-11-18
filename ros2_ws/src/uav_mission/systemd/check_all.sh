#!/bin/bash
# Run all three boot-service checks (read-only): camera, cuVSLAM, Cube link.   check_all.sh
D="$(dirname "$(readlink -f "$0")")"
rc=0
for c in check_camera check_cuvslam check_fclink; do
    echo "=== ${c#check_} ==="
    bash "$D/$c.sh" || rc=1
    echo
done
[ $rc = 0 ] && echo "ALL READY" || echo "SOMETHING IS NOT READY (see FAIL lines above)"
exit $rc
