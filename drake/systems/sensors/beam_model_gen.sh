#!/bin/bash

# Generates the source files for the BeamModelParams vector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
systems=$(dirname "$mydir")
drake=$(dirname "$systems")

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/beam_model_params.named_vector
