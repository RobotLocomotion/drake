#!/bin/bash
#
# Install development and runtime prerequisites for both binary and source
# distributions of Drake on Ubuntu 16.04 (Xenial) or 18.04 (Bionic).

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
#
# Note that the list of packages in binary_distribution/packages.txt is used to
# generate the dependencies of the drake .deb package, so does not include
# development dependencies such as build-essential and cmake.

source "${BASH_SOURCE%/*}/binary_distribution/install_prereqs.sh"

# The following additional dependencies are only needed when developing with
# source distributions.

source "${BASH_SOURCE%/*}/source_distribution/install_prereqs.sh"

# Configure user environment, executing as user if we're under `sudo`.
user_env_script="${BASH_SOURCE%/*}/source_distribution/install_prereqs_user_environment.sh"
if [[ -n "${SUDO_USER:+D}" ]]; then
    sudo -u ${SUDO_USER} bash "${user_env_script}"
else
    source "${user_env_script}"
fi

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
