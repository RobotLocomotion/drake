#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      with_test_only=1
      ;;
    --without-test-only)
      # Ignored for backwards compatibility.
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

# Do not genereate a brew lockfile. This behavior was previously implemented
# with the now deprecated '--no-lock' flag. This env variable allows
# for the behavior to be consistent without using the flag.
export HOMEBREW_BUNDLE_NO_LOCK=1
brew bundle --file="${BASH_SOURCE%/*}/Brewfile"
