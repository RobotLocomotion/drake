#!/bin/bash

# Generates the source files for the
# SchunkWsgCommandReceiverStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::schunk_wsg"

source $drake/tools/lcm_vector_gen.sh

gen_vector "schunk wsg command receiver state vector" \
	   last_target_position trajectory_start_time
