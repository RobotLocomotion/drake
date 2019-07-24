#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if ! command -v /usr/local/bin/brew &>/dev/null; then
  echo 'ERROR: brew is NOT installed. Please ensure that the prerequisites for binary distributions have been installed.' >&2
  exit 4
fi

/usr/local/bin/brew update
/usr/local/bin/brew bundle --file="${BASH_SOURCE%/*}/Brewfile"

if ! command -v /usr/local/bin/pip2 &>/dev/null; then
  echo 'ERROR: pip2 is NOT installed. The post-install step for the python@2 formula may have failed.' >&2
  exit 2
fi

/usr/local/bin/pip2 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"

if ! command -v /usr/local/bin/pip3 &>/dev/null; then
  echo 'ERROR: pip3 is NOT installed. The post-install step for the python formula may have failed.' >&2
  exit 3
fi

/usr/local/bin/pip3 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"
