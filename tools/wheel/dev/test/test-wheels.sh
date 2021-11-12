#!/bin/bash

# This shell script tests the Drake wheels. It takes two arguments; the version
# of the wheel to be tested, and an optional path to the wheels. If the second
# argument is not given, the wheels are assumed to be in the current directory.

set -e

wheel_version="$1"
wheel_dir="$(readlink -f "${2:-$PWD}")"
test_root="$(readlink -f "$(dirname "${BASH_SOURCE}")")"

###############################################################################

test_wheel()
{
    local wheel=drake-$wheel_version-cp$1-cp$1m-manylinux_2_27_x86_64.whl
    local container=pip-drake:test-py$1
    local base=$2
    local py_version=$3

    docker build \
      -t $container \
      --build-arg PLATFORM=$base \
      --build-arg PYTHON=$py_version \
      "$test_root"

    docker run --rm -t \
        -v "$test_root:/test" \
        -v "$wheel_dir:/wheel" \
        $container /test/test-wheel.sh /wheel/$wheel $py_version
}

###############################################################################

if [ -z "$wheel_version" ]; then
    echo "Usage: $0 <version> [<path to wheels>]" >&2
    exit 1
fi

# Test all wheels.
test_wheel 36 ubuntu:18.04 3.6
test_wheel 37 ubuntu:18.04 3.7

# TODO(mwoehlke-kitware) see TODO in build-wheels.sh
# test_wheel 38 ubuntu:20.04 3.8
