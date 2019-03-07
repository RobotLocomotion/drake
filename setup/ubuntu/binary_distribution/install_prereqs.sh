#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 16.04 (Xenial) or 18.04 (Bionic).

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt-get update
apt-get install --no-install-recommends lsb-release

codename=$(lsb_release -sc)

if [[ "${codename}" != 'xenial' && "${codename}" != 'bionic' ]]; then
  echo 'This script requires Ubuntu 16.04 (Xenial) or 18.04 (Bionic)' >&2
  exit 2
fi

apt-get install --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
EOF
)

apt-get install --no-install-recommends $(cat "${BASH_SOURCE%/*}/packages-${codename}.txt" | tr '\n' ' ')
