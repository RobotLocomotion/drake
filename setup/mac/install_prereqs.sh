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

# Note that the version of protobuf here differs from what is in Ubuntu 16.04.
# This turns out to be OK because newer versions of protobuf can generate
# wire-compatible versions for older versions of .proto files, and using the
# default version from Homebrew simplifies the code to find it.
brew install $(tr '\n' ' ' <<EOF
bash-completion
bazel
boost
clang-format
cmake
diffstat
doxygen
dreal-deps/coinor/clp
glew
glib
graphviz
ipopt
libyaml
lz4
nlopt
numpy
patchutils
pkg-config
protobuf
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
matplotlib
pip
pygame
PyYAML
Sphinx
EOF
)
