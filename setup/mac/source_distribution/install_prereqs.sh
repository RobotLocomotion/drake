#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

with_maintainer_only=0
with_test_only=1
with_update=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Install prerequisites that are only needed to run select maintainer
    # scripts. Most developers will not need to install these dependencies.
    --with-maintainer-only)
      with_maintainer_only=1
      ;;
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

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-maintainer-only" --no-lock
fi

if ! command -v pip3.9 &>/dev/null; then
  echo 'ERROR: pip3.9 is NOT installed. The post-install step for the python@3.9 formula may have failed.' >&2
  exit 2
fi

pip3.9 install -r "${BASH_SOURCE%/*}/requirements.txt"

if [[ "${with_test_only}" -eq 1 ]]; then
  pip3.9 install -r "${BASH_SOURCE%/*}/requirements-test-only.txt"
fi

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  pip3.9 install -r "${BASH_SOURCE%/*}/requirements-maintainer-only.txt"
fi
