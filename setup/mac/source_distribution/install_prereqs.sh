#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

with_test_only=1
with_update=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Do NOT install prerequisites that are only needed to build and/or run
    # unit tests, i.e., those prerequisites that are not dependencies of
    # bazel { build, run } //:install.
    --without-test-only)
      with_test_only=0
      ;;
    # Do NOT call brew update during execution of this script.
    --without-update)
      with_update=0
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 5
  esac
  shift
done

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if ! command -v brew &>/dev/null; then
  echo 'ERROR: brew is NOT installed. Please ensure that the prerequisites for binary distributions have been installed.' >&2
  exit 4
fi

if [[ "${with_update}" -eq 1 && "${binary_distribution_called_update:-0}" -ne 1 ]]; then
  brew update || (sleep 30; brew update)
fi

brew bundle --file="${BASH_SOURCE%/*}/Brewfile" --no-lock

if ! command -v pip3.11 &>/dev/null; then
  echo 'ERROR: pip3.11 is NOT installed. The post-install step for the python@3.11 formula may have failed.' >&2
  exit 2
fi

pip3.11 install -r "${BASH_SOURCE%/*}/requirements.txt"

if [[ "${with_test_only}" -eq 1 ]]; then
  pip3.11 install -r "${BASH_SOURCE%/*}/requirements-test-only.txt"
fi
