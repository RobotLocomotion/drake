#!/bin/bash

# Generates the source files for the PendulumStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/pendulum_state.named_vector
gen_vector_proto $mydir/pendulum_input.named_vector
gen_vector_proto $mydir/pendulum_params.named_vector
