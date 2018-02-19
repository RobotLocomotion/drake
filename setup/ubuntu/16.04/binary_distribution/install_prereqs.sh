#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 16.04.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt update
apt install --no-install-recommends $(tr '\n' ' ' <<EOF
apt-transport-https
ca-certificates
wget
EOF
)

wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add
echo 'deb [arch=amd64] https://drake-apt.csail.mit.edu xenial main' > /etc/apt/sources.list.d/drake.list

apt update
apt install --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
EOF
)

apt install --no-install-recommends $(cat "${BASH_SOURCE%/*}/packages.txt" | tr '\n' ' ')
