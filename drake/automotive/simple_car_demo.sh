#!/bin/bash

# Run the simple car demo with its allied apps:
#
#  * steering_command_driver.py for interactive input
#  * drake-visualizer to see things move
#  * bot-spy to see LCM traffic of state and visualization
#  * lcm-logger to capture LCM activity to disk
#
# To kill all the processes, just kill the script in the console with
# Control-C.

set -e

function clean () {
    pkill -P $$
}
trap clean EXIT

me=$(readlink -f $0)
mydir=$(dirname $0)
DRAKE=$(readlink -f $mydir/../..)
DRAKE_DIST=$(readlink -f $DRAKE/..)
DRAKE_DIST_BUILD=${DRAKE_DIST_BUILD:-$DRAKE_DIST/build}
if ! [ -d $DRAKE_DIST_BUILD/install/bin ]; then
    echo "error: $0: cannot find DRAKE_DIST_BUILD at '$DRAKE_DIST_BUILD'"
    exit 1
fi

$DRAKE_DIST_BUILD/install/bin/lcm-logger &
$DRAKE_DIST_BUILD/install/bin/bot-spy &
$DRAKE_DIST_BUILD/install/bin/drake-visualizer &
sleep 1  # Wait, to be sure drake-visualizer sees the load_robot message.
$DRAKE_DIST_BUILD/drake/bin/simple_car_demo &
$mydir/steering_command_driver.py &

wait

# TODO(jwnimmer-tri) Consolidate this script with run_demo_multi_car.sh.
