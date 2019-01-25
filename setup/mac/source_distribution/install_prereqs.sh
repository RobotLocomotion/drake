#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

/usr/local/bin/brew update
/usr/local/bin/brew bundle --file="${BASH_SOURCE%/*}/Brewfile"
/usr/local/bin/brew tap-pin bazelbuild/tap

/usr/local/bin/pip2 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"
/usr/local/bin/pip3 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"
