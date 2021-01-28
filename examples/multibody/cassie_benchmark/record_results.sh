#!/bin/bash
# Collect context information for a benchmark experiment.

${TEST_SRCDIR}/drake/tools/performance/record_results \
    ${TEST_SRCDIR}/drake/examples/multibody/cassie_benchmark/cassie_bench \
    "$@"
