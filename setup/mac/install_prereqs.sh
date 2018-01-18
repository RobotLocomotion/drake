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

brew tap dreal/dreal
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
dreal
gflags
glew
glib
graphviz
ipopt
kcov
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

# We require that Bazel uses the Python installed by Homebrew.
# TODO(jamiesnape): Also support a .bazelrc located in the WORKSPACE.
if [[ ! -f "${HOME}/.bazelrc" ]] || ! grep -q "^build --python_path=" "${HOME}/.bazelrc"; then
  echo "We need to add 'build --python_path=/usr/local/opt/python/libexec/bin/python' to ~/.bazelrc."
  read -r -p "Do you want to continue (y/N)? " reply
  if [[ "${reply}" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
    echo "build --python_path=/usr/local/opt/python/libexec/bin/python" >> "${HOME}/.bazelrc"
  fi
fi

if [[ ! -f "${HOME}/.bazelrc" ]] || ! grep -q "^build --python_path=/usr/local/opt/python/libexec/bin/python$" "${HOME}/.bazelrc"; then
  echo "Using a python other than /usr/local/opt/python/libexec/bin/python is NOT supported" >&2
fi
