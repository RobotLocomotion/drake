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

$DRAKE_DIST/build/bin/lcm-logger &
$DRAKE_DIST/build/bin/bot-spy &
$DRAKE_DIST/build/bin/drake-visualizer &
sleep 1  # Wait, to be sure drake-visualizer sees the load_robot message.
$DRAKE/pod-build/bin/simple_car_demo &
$mydir/steering_command_driver.py &

wait
