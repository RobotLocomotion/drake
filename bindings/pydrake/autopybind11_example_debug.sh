#!/bin/bash
set -eux -o pipefail

stage_dir=${PWD}/tmp/autopybind11
rm -rf ${stage_dir}
mkdir -p ${stage_dir}

output_file=${stage_dir}/output.txt

{
    echo ${output_file}

    # Generate docstrings via mkdoc for reference.
    # N.B. This may be cached, so timing should be ignored for now.
    bazel build //bindings/pydrake:documentation_pybind.h
    # Copy content to avoid issues with Bazel permissions.
    cat ./bazel-bin/bindings/pydrake/documentation_pybind.h > ${stage_dir}/documentation_pybind.h

    # Generate autopybind11 bindings.
    bazel run --run_under="env PYTHONUNBUFFERED=1" \
        //bindings/pydrake:autopybind11_example -- \
        --debug \
        --output_dir=${stage_dir}

} 2>&1 | tee ${output_file}
