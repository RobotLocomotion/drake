#!/bin/bash

# Generates the source files for the Acrobot input, state, and output.
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/acrobot_state_vector.named_vector
