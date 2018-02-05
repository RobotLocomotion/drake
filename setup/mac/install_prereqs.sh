#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.

set -euo pipefail

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.

source "${BASH_SOURCE%/*}/install_prereqs_binary_distribution.sh"

# The following additional dependencies are only needed when developing with
# source distributions.

if ! command -v javac &>/dev/null; then
  echo 'Java JDK is NOT installed' >&2
  exit 4
fi

brew install $(tr '\n' ' ' <<EOF
bash-completion
bazel
clang-format
diffstat
doxygen
graphviz
kcov
patchutils
pkg-config
EOF
)

pip2 install --upgrade $(tr '\n' ' ' <<EOF
matplotlib
pygame
Sphinx
EOF
)

# The preceding only needs to be run once per machine. The following sourced
# script should be run once per user who develops with source distributions.

source "${BASH_SOURCE%/*}/install_prereqs_user_environment.sh"
