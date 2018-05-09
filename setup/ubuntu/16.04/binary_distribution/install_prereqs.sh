#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 16.04.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt-get update
apt-get install --no-install-recommends lsb-release

if [[ "$(lsb_release -sc)" != 'xenial' ]]; then
  echo 'This script requires Ubuntu 16.04 (Xenial)' >&2
  exit 2
fi

# TODO(jwnimmer-tri) Remove this cleanup step sometime after 2018-09-01.
# This script used to install some custom packages directly; here, we'll set
# them to `auto` mode so that Ubuntu will suggest to remove them if nothing
# else is using them.
mark_auto() {
  package="$1"
  shift

  installed=$(dpkg-query --showformat='${Version}\n' --show "${package}" 2>/dev/null || true)
  if [[ -z "${installed}" ]]; then
    return
  fi

  for version in "$@"; do
    if [[ "${version}" == "${installed}" ]]; then
      apt-mark auto "${package}"
      return
    fi
  done
}
mark_auto \
  dreal \
  4.17.12.2 \
  4.17.12.3 \
  4.18.01.3 \
  4.18.02.2 \
  4.18.02.4
mark_auto \
  libibex-dev \
  2.6.3 \
  2.6.5.20180123154310.gitf618c7b296182f90a84d54936d144b87df0747b9~16.04 \
  2.6.5.20180211084215.gitd1419538b4d818ed1cf21a01896bc5eaae5d1d57~16.04

apt-get install --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
EOF
)

apt-get install --no-install-recommends $(cat "${BASH_SOURCE%/*}/packages.txt" | tr '\n' ' ')
