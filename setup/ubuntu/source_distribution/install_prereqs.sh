#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu 18.04 (Bionic) or 20.04 (Focal).
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

apt-get install --no-install-recommends $(tr '\n' ' ' <<EOF
apt-transport-https
ca-certificates
gnupg
wget
EOF
)

codename=$(lsb_release -sc)

# On Bionic, developers must opt-in to kcov support; it comes in with the
# non-standard package name kcov-35 via a Drake-specific PPA.
if [[ "${codename}" == 'bionic' ]] &&
    [[ "$#" -eq 1 ]] &&
    [[ "$1" == "--with-kcov" ]]; then
  wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add
  echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/${codename} ${codename} main" > /etc/apt/sources.list.d/drake.list
  apt-get update
  apt-get install --no-install-recommends kcov-35
fi

apt-get update
apt-get install --no-install-recommends $(cat "${BASH_SOURCE%/*}/packages-${codename}.txt" | tr '\n' ' ')

locale-gen en_US.UTF-8

if [[ "${codename}" == 'focal' ]]; then
  # We need a working /usr/bin/python (of any version).  On Bionic it's there
  # by default, but on Focal we have to ask for it.
  if [[ ! -e /usr/bin/python ]]; then
    apt-get install --no-install-recommends python-is-python3
  else
    echo "/usr/bin/python is already installed"
  fi
fi

dpkg_install_from_wget() {
  package="$1"
  version="$2"
  url="$3"
  checksum="$4"

  # Skip the install if we're already at the exact version.
  installed=$(dpkg-query --showformat='${Version}\n' --show "${package}" 2>/dev/null || true)
  if [[ "${installed}" == "${version}" ]]; then
    echo "${package} is already at the desired version ${version}"
    return
  fi

  # If installing our desired version would be a downgrade, ask the user first.
  if dpkg --compare-versions "${installed}" gt "${version}"; then
    echo "This system has ${package} version ${installed} installed."
    echo "Drake suggests downgrading to version ${version}, our supported version."
    read -r -p 'Do you want to downgrade? [Y/n] ' reply
    if [[ ! "${reply}" =~ ^([yY][eE][sS]|[yY])*$ ]]; then
      echo "Skipping ${package} ${version} installation."
      return
    fi
  fi

  # Download and verify.
  tmpdeb="/tmp/${package}_${version}-amd64.deb"
  wget -O "${tmpdeb}" "${url}"
  if echo "${checksum} ${tmpdeb}" | sha256sum -c -; then
    echo  # Blank line between checkout output and dpkg output.
  else
    echo "ERROR: The ${package} deb does NOT have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

dpkg_install_from_wget \
  bazel 3.0.0 \
  https://releases.bazel.build/3.0.0/release/bazel_3.0.0-linux-x86_64.deb \
  dfa79c10bbfa39cd778e1813a273fd3236beb495497baa046f26d393c58bdc35
