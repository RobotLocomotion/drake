#/bin/bash
# This test should only be run via Bazel, never directly on the command line.

set -ex

# Dump some debugging output.
find .

# Move the originals (which are symlinks) out of the way.
tool_outputs="lcmt_sample_t.lcm sample.cc sample.h sample_translator.cc sample_translator.h"
for item in $tool_outputs; do
    mv drake/tools/test/gen/"$item"{,.orig}
done

# Run the code generator.
# TODO(jwnimmer-tri) De-duplicate this with lcm_vector_gen.sh.
./drake/tools/lcm_vector_gen \
  --lcmtype-dir="drake/tools/test/gen" \
  --cxx-dir="drake/tools/test/gen" \
  --namespace="drake::tools::test" \
  --title="Sample" \
  --workspace=$(pwd) \
  --named_vector_file="drake/tools/test/sample.named_vector"

# Dump some debugging output.
find drake/tools/test/gen

# Re-format the code.
# TODO(jwnimmer-tri) De-duplicate this with lcm_vector_gen.sh.
for item in $tool_outputs; do
    clang-format --style=file -i drake/tools/test/gen/"$item"
done

# Insist that the generated output matches the current copy in git.
for item in ; do
    diff --unified=20 drake/tools/test/gen/"$item"{.orig,}
done

echo "PASS"
