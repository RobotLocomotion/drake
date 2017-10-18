#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in
# SimpleCar.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

namespace="drake::automotive"

source $drake/tools/lcm_vector_gen.sh

gen_lcm_and_vector_from_proto "bicycle car parameters" $mydir/bicycle_car_parameters.named_vector
gen_lcm_and_vector_from_proto "bicycle car state" $mydir/bicycle_car_state.named_vector
gen_lcm_and_vector_from_proto "driving command" $mydir/driving_command_fields.named_vector
gen_vector_proto "idm planner parameters" $mydir/idm_planner_parameters.named_vector
gen_lcm_and_vector_from_proto "maliput railcar state" $mydir/maliput_railcar_state.named_vector
gen_lcm_and_vector_from_proto "maliput railcar params" $mydir/maliput_railcar_params.named_vector
gen_vector_proto "mobil planner parameters" $mydir/mobil_planner_parameters.named_vector
gen_vector_proto "pure pursuit params" $mydir/pure_pursuit_params.named_vector
gen_lcm_and_vector_from_proto "simple car state" $mydir/simple_car_state.named_vector
gen_lcm_and_vector_from_proto "simple car params" $mydir/simple_car_params.named_vector
gen_lcm_and_vector_from_proto "trajectory car state" $mydir/trajectory_car_state.named_vector
gen_vector_proto "trajectory car params" $mydir/trajectory_car_params.named_vector
