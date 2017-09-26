#!/bin/bash

# Generates the source files for the BeamModelParams vector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
systems=$(dirname "$mydir")
drake=$(dirname "$systems")

namespace="drake::systems::sensors"

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto "beam model params" $mydir/beam_model_params.named_vector
