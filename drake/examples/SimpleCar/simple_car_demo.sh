#!/bin/bash

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
