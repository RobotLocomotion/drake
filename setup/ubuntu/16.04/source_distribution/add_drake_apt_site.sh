#!/bin/bash
#
# Add Drake's APT site to sources.list on Ubuntu 16.04.  The APT site is
# optional (not part of the default setup instructions, not required for
# development from source nor use at runtime), but may offer convenient
# installation and upgrades for some developers or users.
#

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add
echo 'deb [arch=amd64] https://drake-apt.csail.mit.edu xenial main' > /etc/apt/sources.list.d/drake.list
