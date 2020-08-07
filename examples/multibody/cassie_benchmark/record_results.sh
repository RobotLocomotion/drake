#!/bin/bash
# Collect context information for a benchmark experiment.
# TODO(rpoyner-tri) find a robust way of recording source code version
# information.

set -e -u -o pipefail

uname -a > ${TEST_UNDECLARED_OUTPUTS_DIR}/kernel.txt || true

(
case $(uname) in
    Linux)
        lsb_release -idrc
        ;;
    Darwin)
        sw_vers
        ;;
    *)
        echo unknown
        ;;
esac
) > ${TEST_UNDECLARED_OUTPUTS_DIR}/os.txt

# TODO(rpoyner-tri) find a robust way of recording compiler settings.
# See discussion in PR #13782 for cautions.

${TEST_SRCDIR}/drake/examples/multibody/cassie_benchmark/cassie_bench \
    --benchmark_report_aggregates_only=true \
    --benchmark_repetitions=9 \
    --benchmark_out_format=json \
    --benchmark_out=${TEST_UNDECLARED_OUTPUTS_DIR}/results.json \
    >& ${TEST_UNDECLARED_OUTPUTS_DIR}/summary.txt

echo Full results are in:
echo ${TEST_UNDECLARED_OUTPUTS_DIR}/
