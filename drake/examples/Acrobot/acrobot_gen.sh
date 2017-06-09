#!/bin/bash

# Generates the source files for the Acrobot input, state, and output.
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::acrobot"

source $drake/tools/lcm_vector_gen.sh

gen_vector "acrobot state vector" theta1 theta2 theta1dot theta2dot
