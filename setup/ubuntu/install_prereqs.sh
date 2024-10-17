#!/bin/bash
#
# Installs development and runtime prerequisites for both binary and source
# distributions of Drake on Ubuntu.

set -euo pipefail

# Check for existence of `SUDO_USER` so that this may be used in Docker
# environments.
if [[ -n "${SUDO_USER:+D}" && $(id -u ${SUDO_USER}) -eq 0 ]]; then
  cat <<eof >&2
It appears that this script is running under sudo, but it was the root user
who ran sudo. That use is not supported; when already running as root, do not
use sudo when calling this script.
eof
  exit 1
fi

at_exit () {
    echo "${me} has experienced an error on line ${LINENO}" \
        "while running the command ${BASH_COMMAND}"
}

me='The Drake source distribution prerequisite setup script'

trap at_exit EXIT

binary_distribution_args=()
source_distribution_args=()

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Set the with/without choices to be appropriate for a Drake Developer.
    --developer)
      source_distribution_args+=(--developer)
      ;;
    # Install prerequisites that are only needed to build documentation, i.e.,
    # those that are dependencies of bazel run //doc:build.
    --with-doc-only)
      source_distribution_args+=(--with-doc-only)
      ;;
    --without-doc-only)
      source_distribution_args+=(--without-doc-only)
      ;;
    # Install bazelisk from a deb package.
    --with-bazel)
      source_distribution_args+=(--with-bazel)
      ;;
    --without-bazel)
      source_distribution_args+=(--without-bazel)
      ;;
    # Install prerequisites that are only needed for --config clang, i.e.,
    # opts-in to the ability to compile Drake's C++ code using Clang.
    --with-clang)
      source_distribution_args+=(--with-clang)
      ;;
    --without-clang)
      source_distribution_args+=(--without-clang)
      ;;
    # Install prerequisites that are only needed to run select maintainer
    # scripts. Most developers will not need to install these dependencies.
    --with-maintainer-only)
      source_distribution_args+=(--with-maintainer-only)
      ;;
    --without-maintainer-only)
      source_distribution_args+=(--without-maintainer-only)
      ;;
    # Install prerequisites that are only needed for tests.
    --with-test-only)
      source_distribution_args+=(--with-test-only)
      ;;
    --without-test-only)
      source_distribution_args+=(--without-test-only)
      ;;
    # Do NOT call apt-get update during execution of this script.
    --without-update)
      binary_distribution_args+=(--without-update)
      ;;
    -y)
      binary_distribution_args+=(-y)
      source_distribution_args+=(-y)
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 1
  esac
  shift
done

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.
#
# Note that the list of packages in binary_distribution/packages.txt is used to
# generate the dependencies of the drake .deb package, so does not include
# development dependencies such as build-essential and cmake.

source "${BASH_SOURCE%/*}/binary_distribution/install_prereqs.sh" \
  "${binary_distribution_args[@]}"

# The following additional dependencies are only needed when developing with
# source distributions.
source "${BASH_SOURCE%/*}/source_distribution/install_prereqs.sh" \
  "${source_distribution_args[@]}"

# Configure user environment, executing as user if we're under `sudo`.
user_env_script="${BASH_SOURCE%/*}/source_distribution/install_prereqs_user_environment.sh"
if [[ -n "${SUDO_USER:+D}" ]]; then
    sudo -u "${SUDO_USER}" bash "${user_env_script}"
else
    source "${user_env_script}"
fi

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
