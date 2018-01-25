#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.

set -euo pipefail

source "${BASH_SOURCE%/*}/install_prereqs_binary_distribution.sh"

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

source "${BASH_SOURCE%/*}/install_prereqs_user_environment.sh"
