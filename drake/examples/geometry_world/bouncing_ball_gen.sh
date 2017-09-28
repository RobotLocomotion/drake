#!/bin/bash

# Generates the source files for the BouncingBallStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::bouncing_ball"

source $drake/tools/lcm_vector_gen.sh

gen_vector "bouncing ball vector" z zdot
