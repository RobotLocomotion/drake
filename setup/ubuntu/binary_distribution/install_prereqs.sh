#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu.

set -euo pipefail

with_update=1
with_asking=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Do NOT call apt-get update during execution of this script.
    --without-update)
      with_update=0
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


if command -v conda &>/dev/null; then
  echo 'NOTE: Drake is not tested regularly with Anaconda, so you may experience compatibility hiccups; when asking for help, be sure to mention that Conda is involved.' >&2
fi

binary_distribution_called_update=0

if [[ "${with_update}" -eq 1 ]]; then
  apt-get update || (sleep 30; apt-get update) || (cat <<EOF && false)
****************************************************************************
Drake is unable to run 'sudo apt-get update', probably because this computer
contains incorrect entries in its sources.list files, or possibly because an
internet service is down.

Run 'sudo apt-get update' and try to resolve whatever problems it reports.
Do not try to set up Drake until that command succeeds.

This is not a bug in Drake.  Do not contact the Drake team for help.
****************************************************************************
EOF

  # Do NOT call apt-get update again when installing prerequisites for source
  # distributions.
  binary_distribution_called_update=1
fi

apt-get install ${maybe_yes} --no-install-recommends lsb-release

codename=$(lsb_release -sc)

if [[ "${codename}" =~ (resolute) ]]; then
  echo 'WARNING: This script is unsupported on Ubuntu 26.04 (Resolute)' >&2
elif ! [[ "${codename}" =~ (jammy|noble) ]]; then
  echo 'ERROR: This script requires Ubuntu 22.04 (Jammy) or 24.04 (Noble)' >&2
  exit 2
fi

apt-get install ${maybe_yes} --no-install-recommends $(cat <<EOF
build-essential
cmake
pkg-config
EOF
)

packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}.txt")
apt-get install ${maybe_yes} --no-install-recommends ${packages}
