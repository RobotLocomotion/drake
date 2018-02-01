#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

if ! command -v java &>/dev/null; then
  echo 'Java JRE is NOT installed' >&2
  exit 2
fi

if ! command -v brew &>/dev/null; then
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi

if [[ ! -f /usr/include/expat.h || ! -f /usr/include/zlib.h ]]; then
  echo 'Command Line Tools for Xcode are NOT installed' >&2
  exit 3
fi

brew update

brew tap dreal/dreal
brew tap robotlocomotion/director

brew update
brew upgrade

brew install $(tr '\n' ' ' <<EOF
boost
cmake
dreal
gflags
glew
glib
ipopt
libyaml
lz4
nlopt
numpy
protobuf
python
scipy
tinyxml
tinyxml2
vtk@8.0
yaml-cpp@0.6
EOF
)

pip2 install --upgrade $(tr '\n' ' ' <<EOF
lxml
pip
PyYAML
EOF
)
