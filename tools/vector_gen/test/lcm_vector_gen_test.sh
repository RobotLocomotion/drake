# This test should only be run via Bazel, never directly on the command line.
# Unit test for lcm_vector_gen.py.

set -ex

[[ -f ./tools/vector_gen/lcm_vector_gen_test ]]  # Fail-fast when not under Bazel.

# Dump some debugging output.
find .

# These are the filenames under test.
tool_outputs="lcmt_sample_t.lcm sample.cc sample.h sample_translator.cc 
  sample_translator.h"

# Insist that the generated output matches the goal copy in git.
mv tools/vector_gen/test/lcmt_sample_t.lcm tools/vector_gen/test/gen/
for item in $tool_outputs; do
  diff --unified=20 tools/vector_gen/test/{goal,gen}/"${item}"
done

echo "PASS"
