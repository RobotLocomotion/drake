#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

with_developer=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      with_developer=1
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

brew bundle --file="${BASH_SOURCE%/*}/Brewfile"

if [[ "${with_developer}" -eq 1 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-developer"
fi
