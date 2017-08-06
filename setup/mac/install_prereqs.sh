#!/bin/bash
#
# Prerequisite set-up script for a Drake build with Bazel on macOS or OS X.

set -eu

if [[ $EUID -eq 0 ]]; then
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

brew tap homebrew/science
brew tap robotlocomotion/director

brew update
brew upgrade

brew install $(tr '\n' ' ' <<EOF

bazel
boost
clang-format
doxygen
gcc
glib
numpy
patchutils
pkg-config
python
tinyxml
vtk@8.0
zlib

EOF
)

pip2 install --upgrade Sphinx
