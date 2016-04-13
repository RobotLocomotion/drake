#!/bin/bash

# Generate the source files for LCM Vector concept classes used in SimpleCar.

me=$(readlink -f $0)
mydir=$(dirname $me)
drake=$(dirname $(dirname $mydir))

# Call the code generator with common configuration.
# @param1 title -- used to create class/type names
# @param... --- used to create field names for vector entries
gen () {
    title="$1"
    shift
    $mydir/lcm_vector_gen.py \
        --lcmtype-dir=$drake/lcmtypes \
        --header-dir=$mydir \
        --title="$title" "$@"
}

gen "simple car state" x y heading velocity
gen "driving command" steering_angle throttle brake
gen "euler floating joint state" x y z roll pitch yaw
