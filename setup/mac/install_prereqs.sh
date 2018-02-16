#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.

set -euxo pipefail

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.

source "${BASH_SOURCE%/*}/binary_distribution/install_prereqs.sh"

# The following additional dependencies are only needed when developing with
# source distributions.

brew bundle --file="${BASH_SOURCE%/*}/Brewfile"

pip2 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"

# The preceding only needs to be run once per machine. The following sourced
# script should be run once per user who develops with source distributions.

source "${BASH_SOURCE%/*}/install_prereqs_user_environment.sh"
