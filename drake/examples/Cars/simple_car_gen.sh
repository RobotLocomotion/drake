#!/bin/bash

# Generate the source files for LCM Vector concept classes used in SimpleCar.

me=$(readlink -f $0)
mydir=$(dirname $me)
drake=$(dirname $(dirname $mydir))

CLANG_FORMAT=${CLANG_FORMAT:-clang-format-3.6}

# Call the code generator with common configuration.
# @param1 title -- used to create class/type names
# @param... --- used to create field names for vector entries
gen () {
    title="$1"
    shift
    $mydir/lcm_vector_gen.py \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --title="$title" "$@"
}

gen "driving command" steering_angle throttle brake
gen "euler floating joint state" x y z roll pitch yaw
gen "idm with trajectory agent state" x_e v_e x_a v_a a_a
gen "simple car state" x y heading velocity

$CLANG_FORMAT --style=file -i \
  $mydir/gen/driving_command.h \
  $mydir/gen/driving_command.cc \
  $mydir/gen/driving_command_translator.h \
  $mydir/gen/driving_command_translator.cc \
  $mydir/gen/euler_floating_joint_state.h \
  $mydir/gen/euler_floating_joint_state.cc \
  $mydir/gen/euler_floating_joint_state_translator.h \
  $mydir/gen/euler_floating_joint_state_translator.cc \
  $mydir/gen/idm_with_trajectory_agent_state.h \
  $mydir/gen/idm_with_trajectory_agent_state.cc \
  $mydir/gen/idm_with_trajectory_agent_state_translator.h \
  $mydir/gen/idm_with_trajectory_agent_state_translator.cc \
  $mydir/gen/simple_car_state.h \
  $mydir/gen/simple_car_state.cc \
  $mydir/gen/simple_car_state_translator.h \
  $mydir/gen/simple_car_state_translator.cc
