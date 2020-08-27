#!/bin/bash
# Collect context information for a benchmark experiment.
# TODO(rpoyner-tri) find a robust way of recording source code version
# information.

set -e -u -o pipefail

uname -a > ${TEST_UNDECLARED_OUTPUTS_DIR}/kernel.txt || true

# Fill this in with a platform-specific command to control processor affinity,
# if any.
AFFINITY_COMMAND=""

case $(uname) in
    Linux)
        lsb_release -idrc
        # Choosing processor #0 is arbitrary. It is up to experimenters
        # to ensure it is reliably idle during experiments.
        AFFINITY_COMMAND="taskset 0x1"
        ;;
    Darwin)
        sw_vers
        ;;
    *)
        echo unknown
        ;;
esac > ${TEST_UNDECLARED_OUTPUTS_DIR}/os.txt

${TEST_SRCDIR}/drake/tools/workspace/cc/identify_compiler \
 > ${TEST_UNDECLARED_OUTPUTS_DIR}/compiler.txt

${AFFINITY_COMMAND} \
${TEST_SRCDIR}/drake/examples/multibody/cassie_benchmark/cassie_bench \
    --benchmark_display_aggregates_only=true \
    --benchmark_repetitions=9 \
    --benchmark_out_format=json \
    --benchmark_out=${TEST_UNDECLARED_OUTPUTS_DIR}/results.json \
    "$@" \
    >& ${TEST_UNDECLARED_OUTPUTS_DIR}/summary.txt

echo Full results are in:
echo ${TEST_UNDECLARED_OUTPUTS_DIR}/
