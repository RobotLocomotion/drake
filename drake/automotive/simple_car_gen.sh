#!/bin/bash

# Generates the source files for LCM messages and BasicVectors used in 
# SimpleCar.

me=$(readlink -f "$0")
mydir=$(dirname "$me")
drake=$(dirname "$mydir")

namespace="drake::automotive"

source $drake/tools/lcm_vector_gen.sh

gen_lcm_and_vector "driving command" steering_angle throttle brake
gen_lcm_and_vector "euler floating joint state" x y z roll pitch yaw
gen_lcm_and_vector "idm with trajectory agent state" x_e v_e x_a v_a a_a
gen_lcm_and_vector "simple car state" x y heading velocity
gen_lcm_and_vector "simple car config" wheelbase track max_abs_steering_angle max_velocity max_acceleration velocity_lookahead_time velocity_kp

gen_vector_yaml "idm with trajectory agent parameters" $drake/automotive/idm_with_trajectory_agent_parameters.yaml
