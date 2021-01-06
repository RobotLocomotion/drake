#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 18.04 (Bionic) or 20.04 (Focal).

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported for building and using the Drake Python bindings' >&2
fi

apt-get update
apt-get install --no-install-recommends lsb-release

codename=$(lsb_release -sc)

if [[ "${codename}" != 'bionic' && "${codename}" != 'focal' ]]; then
  echo 'ERROR: This script requires Ubuntu 18.04 (Bionic) or 20.04 (Focal)' >&2
  exit 2
fi

apt-get install --no-install-recommends $(cat <<EOF
build-essential
cmake
pkg-config
EOF
)

packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}.txt")
apt-get install --no-install-recommends ${packages}
