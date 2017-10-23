#!/bin/bash

# Usage: source $drake/tools/lcm_vector_gen.sh

# Definitions for auto-generating Drake BasicVectors, LCM types, and
# translators between the two.

# Requires that the variables drake and mydir are already defined.
# $drake is the top of the drake source tree, i.e. $drake/tools contains this
#        script.
# $mydir/gen is the directory to which the generated files to be written.

if [ -z "$drake" ] || [ -z "$mydir" ]
then
  echo "Required input missing."
  exit 1
fi

workspace=$(dirname "$drake")

mkdir -p $drake/lcmtypes
mkdir -p $mydir/gen

# Call the code generator to produce an LCM message, a translator, and
# a Drake BasicVector based on specifications contained in a NamedVector proto.
#
# @param named_vector_file --- the file containing a text NamedVector proto.
gen_lcm_and_vector_from_proto() {
    named_vector_file="$1"
    shift
    bazel run //drake/tools:lcm_vector_gen -- \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --workspace="$workspace" \
        --named_vector_file="$named_vector_file"
}

# Call the code generator to produce just a Drake BasicVector based on a
# NamedVector proto spec.
#
# @param named_vector_file --- the NamedVector specification of vector fields
gen_vector_proto () {
    named_vector_file="$1"
    bazel run //drake/tools:lcm_vector_gen -- \
        --cxx-dir=$mydir/gen \
        --workspace="$workspace" \
        --named_vector_file="$named_vector_file"
}
