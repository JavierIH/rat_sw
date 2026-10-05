#!/bin/bash
# tools/straight_battery.sh speed [speed...]: for each speed, a 180 in place
# (CAL TURN 2) and a CAL STRAIGHT 3 at that speed, then the centring summary
# of the recording (calib_analyze.py). For the centring tests on a ring
# (layout I): the robot shuttles along one 4-cell side.
cd "$(dirname "$0")/.."
for v in "$@"; do
    tools/robot.sh -w "data saved" -t 25 "CAL TURN 2" | grep -E "!!"
    tools/robot.sh -w "data saved" -t 25 "CAL STRAIGHT 3 $v" | grep -E "forward|!!"
    python3 tools/calib_analyze.py "$(ls -t tools/calib_data/*straight*.csv | head -1)" | grep -E "centring:|per 45"
done
