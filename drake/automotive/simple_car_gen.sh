#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in
# SimpleCar.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/bicycle_car_parameters.named_vector
gen_lcm_and_vector_from_proto $mydir/bicycle_car_state.named_vector
gen_lcm_and_vector_from_proto $mydir/driving_command.named_vector
gen_vector_proto $mydir/idm_planner_parameters.named_vector
gen_lcm_and_vector_from_proto $mydir/maliput_railcar_state.named_vector
gen_vector_proto $mydir/maliput_railcar_params.named_vector
gen_vector_proto $mydir/mobil_planner_parameters.named_vector
gen_vector_proto $mydir/pure_pursuit_params.named_vector
gen_lcm_and_vector_from_proto $mydir/simple_car_state.named_vector
gen_vector_proto $mydir/simple_car_params.named_vector
gen_lcm_and_vector_from_proto $mydir/trajectory_car_state.named_vector
gen_vector_proto $mydir/trajectory_car_params.named_vector
