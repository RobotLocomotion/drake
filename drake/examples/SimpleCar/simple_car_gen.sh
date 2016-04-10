#!/bin/bash

me=$(readlink -f $0)
mydir=$(dirname $me)
drake=$(dirname $(dirname $mydir))

gen () {
    title="$1"
    shift
    $mydir/lcm_vector_gen.py \
        --lcmtype-dir=$drake/lcmtypes \
        --header-dir=$mydir \
        --title="$title" "$@"
}

gen "simple car state" x y heading steering_angle velocity
gen "driving command" steering_angle throttle brake
