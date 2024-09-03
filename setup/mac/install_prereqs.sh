#!/bin/bash
#
# Install development and runtime prerequisites for both binary and source
# distributions of Drake on macOS.

set -euxo pipefail

binary_distribution_args=(--without-python-dependencies)
source_distribution_args=()

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Do NOT install prerequisites that are only needed to build and/or run
    # unit tests, i.e., those prerequisites that are not dependencies of
    # bazel { build, run } //:install.
    --without-test-only)
      source_distribution_args+=(--without-test-only)
      ;;
    # Do NOT call brew update during execution of this script.
    --without-update)
      binary_distribution_args+=(--without-update)
      source_distribution_args+=(--without-update)
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 1
  esac
  shift
done

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.

# N.B. We need `${var:-}` here because mac's older version of bash does
# not seem to be able to cope with an empty array.

source "${BASH_SOURCE%/*}/binary_distribution/install_prereqs.sh" \
  "${binary_distribution_args[@]:-}"

# The following additional dependencies are only needed when developing with
# source distributions.

source "${BASH_SOURCE%/*}/source_distribution/install_prereqs.sh" \
  "${source_distribution_args[@]:-}"

# The preceding only needs to be run once per machine. The following sourced
# script should be run once per user who develops with source distributions.

source "${BASH_SOURCE%/*}/source_distribution/install_prereqs_user_environment.sh"

# Bootstrap the venv. The `setup` is required here (nothing else runs it), but
# the `sync` is not strictly requied (bazel will run it for us). We do it here
# anyway so that the user gets immediate, streaming feedback about problems
# with internet downloads, or other kinds of glitches. (When run from Bazel,
# stderr text is batched up until much later, not streamed.)

"${BASH_SOURCE%/*}/../../tools/workspace/venv/setup"
"${BASH_SOURCE%/*}/../../tools/workspace/venv/sync"
