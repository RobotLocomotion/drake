#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

developer=0
with_asking=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      developer=1
      ;;
    # Pass -y along to apt-get.
    -y)
      with_asking=0
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 3
  esac
  shift
done

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

if [[ "${with_asking}" -eq 0 ]]; then
  maybe_yes='-y'
else
  maybe_yes=''
fi

apt-get install ${maybe_yes} --no-install-recommends $(cat <<EOF
wget
EOF
)

. /etc/os-release

packages=$(cat "${BASH_SOURCE%/*}/packages-${VERSION_CODENAME}-build.txt")
apt-get install ${maybe_yes} --no-install-recommends ${packages}

# Ensure that we have available a locale that supports UTF-8 for generating a
# C++ header containing Python API documentation during the build.
apt-get install ${maybe_yes} --no-install-recommends locales
locale-gen en_US.UTF-8

# We need a working /usr/bin/python (of any version).
if [[ ! -e /usr/bin/python ]]; then
  apt-get install ${maybe_yes} --no-install-recommends python-is-python3
else
  echo "/usr/bin/python is already installed"
fi

if [[ "${developer}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${VERSION_CODENAME}-developer.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
  source "${BASH_SOURCE%/*}/install_bazelisk.sh"
  if [[ "${VERSION_CODENAME}" == "noble" ]]; then
    "${BASH_SOURCE%/*}/install_kcov.sh"
  fi
fi

# On Noble, Drake doesn't install anything related to GCC 14, but if the user
# has chosen to install some GCC 14 libraries but has failed to install all of
# them correctly as a group, Drake's documentation header file parser will fail
# with a libclang-related complaint. Therefore, we'll help the user clean up
# their mess, to avoid apparent Drake build errors.
if [[ "${VERSION_CODENAME}" == "noble" ]]; then
  status=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgcc-14-dev 2>/dev/null || true)
  if [[ "${status}" == "ii " ]]; then
    status_stdcxx=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libstdc++-14-dev 2>/dev/null || true)
    status_fortran=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgfortran-14-dev 2>/dev/null || true)
    if [[ "${status_stdcxx}" != "ii " || "${status_fortran}" != "ii " ]]; then
      apt-get install ${maybe_yes} --no-install-recommends libgcc-14-dev libstdc++-14-dev libgfortran-14-dev
    fi
  fi
fi
