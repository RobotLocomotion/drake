#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in
# SimpleCar.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

namespace="drake::automotive"

source $drake/tools/lcm_vector_gen.sh

gen_lcm_and_vector_from_proto "bicycle parameters" $drake/automotive/bicycle_parameters.named_vector
gen_lcm_and_vector_from_proto "driving command" $drake/automotive/driving_command_fields.named_vector
gen_lcm_and_vector_from_proto "endless road car config" $drake/automotive/dev/endless_road_car_config.named_vector
gen_lcm_and_vector_from_proto "endless road car state" $drake/automotive/dev/endless_road_car_state.named_vector
gen_lcm_and_vector_from_proto "endless road oracle output" $drake/automotive/dev/endless_road_oracle_output.named_vector
gen_lcm_and_vector_from_proto "euler floating joint state" $drake/automotive/euler_floating_joint_state.named_vector
gen_vector_proto "idm planner parameters" $drake/automotive/idm_planner_parameters.named_vector
gen_lcm_and_vector_from_proto "simple car state" $drake/automotive/simple_car_state.named_vector
gen_lcm_and_vector_from_proto "simple car config" $drake/automotive/simple_car_config.named_vector
