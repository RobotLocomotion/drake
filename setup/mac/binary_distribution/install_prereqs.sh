#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

if ! command -v /usr/local/bin/brew &>/dev/null; then
  /usr/bin/ruby -e "$(/usr/bin/curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi

/usr/local/bin/brew update
/usr/local/bin/brew bundle --file="${BASH_SOURCE%/*}/Brewfile"

if [[ ! -f /usr/include/expat.h || ! -f /usr/include/zlib.h ]]; then
  /usr/bin/xcode-select --install
fi

/usr/local/bin/pip2 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"
