#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in
# SimpleCar.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

namespace="drake::automotive"

source $drake/tools/lcm_vector_gen.sh

gen_lcm_and_vector_from_proto "bicycle car parameters" $drake/automotive/bicycle_car_parameters.named_vector
gen_lcm_and_vector_from_proto "bicycle car state" $drake/automotive/bicycle_car_state.named_vector
gen_lcm_and_vector_from_proto "driving command" $drake/automotive/driving_command_fields.named_vector
gen_lcm_and_vector_from_proto "euler floating joint state" $drake/automotive/euler_floating_joint_state.named_vector
gen_vector_proto "idm planner parameters" $drake/automotive/idm_planner_parameters.named_vector
gen_lcm_and_vector_from_proto "maliput railcar state" $drake/automotive/maliput_railcar_state.named_vector
gen_lcm_and_vector_from_proto "maliput railcar params" $drake/automotive/maliput_railcar_params.named_vector
gen_vector_proto "mobil planner parameters" $drake/automotive/mobil_planner_parameters.named_vector
gen_lcm_and_vector_from_proto "simple car state" $drake/automotive/simple_car_state.named_vector
gen_lcm_and_vector_from_proto "simple car params" $drake/automotive/simple_car_params.named_vector
