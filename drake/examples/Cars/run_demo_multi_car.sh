#!/bin/bash

# Run the multi-car demo with its allied apps:
#
#  * drake-visualizer to see things move
#  * bot-spy to see LCM traffic of state and visualization
#
# To kill all the processes, just kill the script in the console with
# Control-C.
#
# The first command-line argument, if supplied, must be readable as an
# integer and it sets the number of vehicles to N.  (The default N is
# 100; minimum N is 1.)

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

$DRAKE_DIST_BUILD/install/bin/bot-spy &
$DRAKE_DIST_BUILD/install/bin/drake-visualizer &
sleep 1  # Wait, to be sure drake-visualizer sees the load_robot message.
$DRAKE_DIST_BUILD/drake/bin/demo_multi_car $1 &

wait

# TODO(jwnimmer-tri) Consolidate this script with simple_car_demo.sh.
