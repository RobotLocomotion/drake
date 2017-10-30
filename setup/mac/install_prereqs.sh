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
pip
pygame
PyYAML
Sphinx
EOF
)

# Needed to allow 'import google.protobuf.text_format' on macOS with the
# protobuf homebrew bottle
USER=$( id -un )
PY_SITE_PKGS=/Users/$USER/Library/Python/2.7/lib/python/site-packages
mkdir -p $PY_SITE_PKGS
echo 'import site; site.addsitedir("/usr/local/lib/python2.7/site-packages")' >> $PY_SITE_PKGS/homebrew.pth
