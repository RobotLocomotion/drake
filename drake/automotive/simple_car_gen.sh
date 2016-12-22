#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in
# SimpleCar.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

namespace="drake::automotive"

source $drake/tools/lcm_vector_gen.sh

gen_lcm_and_vector_from_yaml "driving command" $drake/automotive/driving_command_fields.yaml
gen_lcm_and_vector "euler floating joint state" x y z roll pitch yaw
gen_vector_yaml "idm planner parameters" $drake/automotive/idm_planner_parameters.yaml
gen_lcm_and_vector "simple car state" x y heading velocity
gen_lcm_and_vector "simple car config" wheelbase track max_abs_steering_angle max_velocity max_acceleration velocity_lookahead_time velocity_kp
