# This test should only be run via Bazel, never directly on the command line.
# Unit test for lcm_vector_gen.py.

set -ex

[[ -f ./drake/tools/lcm_vector_gen_test ]]  # Fail-fast when not under Bazel.

# Dump some debugging output.
find .

# These are the filenames under test.
tool_outputs="lcmt_sample_t.lcm sample.cc sample.h sample_translator.cc 
  sample_translator.h"

# Insist that the generated output matches the goal copy in git.
mv drake/tools/test/lcmt_sample_t.lcm drake/tools/test/gen/
for item in $tool_outputs; do
  diff --unified=20 drake/tools/test/{goal,gen}/"${item}"
done

echo "PASS"
