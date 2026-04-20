#!/bin/bash
#
# Install development and runtime prerequisites for both binary and source
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

binary_args=()
build_args=()
prefetch_bazel=0
user_environment_only=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      build_args+=(--developer)
      prefetch_bazel=1
      ;;
    --user-environment-only)
      user_environment_only=1
      ;;
    # Do NOT call apt-get update during execution of this script.
    --without-update)
      binary_args+=(--without-update)
      ;;
    -y)
      binary_args+=(-y)
      build_args+=(-y)
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 1
  esac
  shift
done

if [[ ${user_environment_only} -eq 0 ]]; then
  # Dependencies that are installed by the following sourced script that are
  # needed when developing with binary distributions are also needed when
  # developing with source distributions.

  source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]}"

  # The following additional dependencies are only needed when developing with
  # source distributions.
  source "${BASH_SOURCE%/*}/install_prereqs_build.sh" "${build_args[@]}"
fi

# Configure user environment, executing as user if we're under `sudo`.
user_env_script="${BASH_SOURCE%/*}/install_prereqs_user_environment.sh"
user_env_script_args=()
if [[ ${prefetch_bazel} -eq 1 ]]; then
  user_env_script_args+=(--prefetch-bazel)
fi
if [[ -n "${SUDO_USER:+D}" ]]; then
    sudo -u "${SUDO_USER}" bash "${user_env_script}" "${user_env_script_args[@]}"
else
    source "${user_env_script}" "${user_env_script_args[@]}"
fi

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
