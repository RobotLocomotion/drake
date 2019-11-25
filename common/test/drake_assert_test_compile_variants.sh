#!/bin/bash

# This test should only be invoked via Bazel.
set -ex

find .  # Get some debugging output.

capture_cc_env="$1"
drake_assert_test_compile_cc="$2"

# TODO(jamiesnape): Determine this information from Bazel.
if [[ "$(uname -s)" == Darwin ]]; then
  export DEVELOPER_DIR="$(xcode-select --print-path)"
  export SDKROOT="$(xcrun --show-sdk-path)"
fi

# Make sure we know what C++ compiler to use.
source "$capture_cc_env"
[[ ! -z "$BAZEL_CC" ]]
BAZEL_CC=$(python3 -c 'import os; print(os.path.realpath("'"$BAZEL_CC"'"))')
command -v "$BAZEL_CC"
"$BAZEL_CC" --version

# TODO(#6996) Do this unconditionally once #6996 is fully merged.
if [[ ! -e ./drake ]]; then
  # Mimic `include_prefix = "drake"` per drake_cc_library's default.
  ln -s . drake
fi

# Confirm that it compiles successfully, whether or not assertions are enabled.
TESTING_CXXFLAGS="$BAZEL_CC_FLAGS -std=c++17 -I . \
    -c $drake_assert_test_compile_cc \
    -o /dev/null"
"$BAZEL_CC" $TESTING_CXXFLAGS
"$BAZEL_CC" $TESTING_CXXFLAGS -DDRAKE_ENABLE_ASSERTS -UDRAKE_DISABLE_ASSERTS
"$BAZEL_CC" $TESTING_CXXFLAGS -UDRAKE_ENABLE_ASSERTS -DDRAKE_DISABLE_ASSERTS

# Setting -DDRAKE_ASSERT_TEST_COMPILE_ERROR1 should cause a compiler error.
TESTING_CXXFLAGS1="$TESTING_CXXFLAGS -DDRAKE_ASSERT_TEST_COMPILE_ERROR1"
! "$BAZEL_CC" $TESTING_CXXFLAGS1
! "$BAZEL_CC" $TESTING_CXXFLAGS1 -DDRAKE_ENABLE_ASSERTS -UDRAKE_DISABLE_ASSERTS
! "$BAZEL_CC" $TESTING_CXXFLAGS1 -UDRAKE_ENABLE_ASSERTS -DDRAKE_DISABLE_ASSERTS

# Setting -DDRAKE_ASSERT_TEST_COMPILE_ERROR2 should cause a compiler error.
TESTING_CXXFLAGS2="$TESTING_CXXFLAGS -DDRAKE_ASSERT_TEST_COMPILE_ERROR2"
! "$BAZEL_CC" $TESTING_CXXFLAGS2
! "$BAZEL_CC" $TESTING_CXXFLAGS2 -DDRAKE_ENABLE_ASSERTS -UDRAKE_DISABLE_ASSERTS
! "$BAZEL_CC" $TESTING_CXXFLAGS2 -UDRAKE_ENABLE_ASSERTS -DDRAKE_DISABLE_ASSERTS
