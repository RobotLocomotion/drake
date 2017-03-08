#!/bin/bash

# This program re-generates the source files in this directory.
# This program is locally invoked by a developer, not `bazel run`.
# This program's changes to the generated source files should be committed.

# Our weird filename (leading underscore) is to help disambiguate (i.e., sort)
# generated files from hand-written files.

set -e

# N.B. $mydir is our _parent_ directory (`test`), not this directory (`gen`).
# TODO(jwnimmer-tri) Refactor lcm_vector_gen.sh to have a better mnemonic.
mydir=$(python -c \
    'import os.path as p; print(p.dirname(p.dirname(p.realpath("'"$0"'"))))')
drake=$(python -c \
    'import os.path as p; print(p.realpath("'"$mydir"'/../.."))')
namespace="drake::tools::test"

source $drake/tools/lcm_vector_gen.sh

# Generate one canonical test vector type.
gen_lcm_and_vector_from_proto "sample" $mydir/sample.named_vector

# Until CMake stops forcing us to dump all the lcmtypes into one place, the
# lcm_vector_gen has to place lcmt_sample_t.lcm in a silly place.  Since nobody
# needs it to be there, we'll move it manually for now.
mv $drake/lcmtypes/lcmt_sample_t.lcm $mydir/gen/
