#!/bin/bash
#
# Install development and runtime prerequisites for both binary and source
# distributions of Drake on Ubuntu 16.04.

set -euo pipefail

at_exit () {
    echo "${me} has experienced an error on line ${LINENO}" \
        "while running the command ${BASH_COMMAND}"
}

me='The Drake source distribution prerequisite setup script'

trap at_exit EXIT

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.

source "${BASH_SOURCE%/*}/binary_distribution/install_prereqs.sh"

# The following additional dependencies are only needed when developing with
# source distributions.

source "${BASH_SOURCE%/*}/source_distribution/install_prereqs.sh"

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
