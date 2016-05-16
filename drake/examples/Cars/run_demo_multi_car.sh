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
# integer and it sets the number of vehicles to N+3.  (The default N is
# 100; minimum N is 0.)

set -e

function clean () {
    pkill -P $$
}
trap clean EXIT

me=$(readlink -f $0)
mydir=$(dirname $0)
DRAKE=$(readlink -f $mydir/../..)
DRAKE_DIST=$(readlink -f $DRAKE/..)

$DRAKE_DIST/build/bin/bot-spy &
$DRAKE_DIST/build/bin/drake-visualizer &
sleep 1  # Wait, to be sure drake-visualizer sees the load_robot message.
$DRAKE/pod-build/bin/demo_multi_car $1 &

wait

# TODO(jwnimmer) Consolidate this script with simple_car_demo.sh.
