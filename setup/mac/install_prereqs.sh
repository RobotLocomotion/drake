#!/bin/bash
#
# Prerequisite set-up script for a Drake build on Mac.

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "This script must NOT be run as root" >&2
  exit 1
fi

if ! command -v javac &>/dev/null; then
  echo "Java is NOT installed" >&2
  exit 2
fi

if ! command -v brew &>/dev/null; then
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi

if [[ ! -f /usr/include/expat.h || ! -f /usr/include/zlib.h ]]; then
  echo "Command Line Tools for Xcode are NOT installed" >&2
  exit 3
fi

brew tap homebrew/science
brew tap robotlocomotion/director

brew update
brew upgrade

brew install $(tr '\n' ' ' <<EOF
bash-completion
bazel
boost
clang-format
cmake
diffstat
doxygen
glew
glib
graphviz
ipopt
libyaml
lz4
numpy
patchutils
pkg-config
protobuf@2.6
python
scipy
tinyxml
tinyxml2
vtk@8.0
yaml-cpp
EOF
)

pip2 install --upgrade $(tr '\n' ' ' <<EOF
lxml
pip
pygame
PyYAML
Sphinx
EOF
)
