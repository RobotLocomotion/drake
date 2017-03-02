#/bin/bash
# This test should only be run via Bazel, never directly on the command line.

set -ex

mkdir tmp

# Dump some debugging output.
find .

# Run the code generator.
# TODO(jwnimmer-tri) De-duplicate this with lcm_vector_gen.sh.
./drake/tools/lcm_vector_gen \
  --lcmtype-dir="tmp" \
  --cxx-dir="tmp" \
  --namespace="drake::tools::test" \
  --title="Sample" \
  --workspace=$(pwd) \
  --named_vector_file="drake/tools/test/sample.named_vector"

# Dump some debugging output.
find tmp

# Re-format the code.
# TODO(jwnimmer-tri) De-duplicate this with lcm_vector_gen.sh.
clang-format --style=file -i tmp/*.h tmp/*.cc

# Insist that the generated output matches the current copy in git.
for item in lcmt_sample_t.lcm sample.cc sample.h sample_translator.cc sample_translator.h; do
    # Fuzz out a desirable difference in the generated code.
    sed -i -e 's|include "tmp|include "drake/tools/test/gen|' tmp/"$item"
    # The files should be identical now.
    diff --unified=20 drake/tools/test/gen/"$item" tmp/"$item"
done

echo "PASS"
