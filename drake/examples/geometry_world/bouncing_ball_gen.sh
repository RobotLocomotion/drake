#!/bin/bash

# Generates the source files for the BouncingBallStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/bouncing_ball_vector.named_vector
