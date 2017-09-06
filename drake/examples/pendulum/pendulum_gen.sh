#!/bin/bash

# Generates the source files for the PendulumStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::pendulum"

source $drake/tools/lcm_vector_gen.sh

gen_vector "pendulum state" theta thetadot
gen_vector "pendulum input" tau
gen_vector_proto "pendulum params" $drake/examples/pendulum/pendulum_params.named_vector
