#!/bin/bash

# Generates the source files for the
# SchunkWsgTrajectoryGeneratorStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
manipulation=$(dirname "$mydir")
drake=$(dirname "$manipulation")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto \
  $mydir/schunk_wsg_trajectory_generator_state_vector.named_vector
