#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported. Please remove the Anaconda bin directory from the PATH.' >&2
fi

if ! command -v /usr/local/bin/brew &>/dev/null; then
  /usr/bin/ruby -e "$(/usr/bin/curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi

/usr/local/bin/brew update
# TODO(jamiesnape): Remove line uninstalling embree, ospray, and vtk@8.1
# formulae on or after 2019-07-01.
/usr/local/bin/brew uninstall --force embree ospray vtk@8.1
# TODO(jamiesnape): Remove line uninstalling ipopt on or after 2019-08-01.
/usr/local/bin/brew uninstall --force ipopt
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
