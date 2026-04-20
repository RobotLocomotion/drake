#!/bin/bash
#
# Install development and runtime prerequisites for both binary and source
# distributions of Drake on macOS.

set -euxo pipefail

binary_args=(--without-python-dependencies)
build_args=()
user_environment_only=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      build_args+=(--developer)
      ;;
    --user-environment-only)
      user_environment_only=1
      ;;
    # Do NOT call brew update during execution of this script.
    --without-update)
      binary_args+=(--without-update)
      ;;
    # Ignored for compatibility with Ubuntu.
    -y)
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 1
  esac
  shift
done

if [[ ${user_environment_only} -eq 0 ]]; then
  # Dependencies that are installed by the following sourced script that are
  # needed when developing with binary distributions are also needed when
  # developing with source distributions.

  # N.B. We need `${var:-}` here because mac's older version of bash does
  # not seem to be able to cope with an empty array.

  source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]:-}"

  # The following additional dependencies are only needed when developing with
  # source distributions.

  source "${BASH_SOURCE%/*}/install_prereqs_build.sh" "${build_args[@]:-}"
fi

# The preceding only needs to be run once per machine. The following sourced
# script should be run once per user who develops with source distributions.

source "${BASH_SOURCE%/*}/install_prereqs_user_environment.sh"
