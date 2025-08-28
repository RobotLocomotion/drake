#!/bin/bash
#
# On Ubuntu 24.04 Noble, installs kcov from a drake-mirror download. (For Ubuntu
# 22.04 Jammy, the packages-jammy-test-only.txt already has kcov, so this script
# should not be used.)
#
# This script does not accept any command line arguments.

set -euo pipefail

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

# Install kcov. Because Noble does not offer kcov natively, this file was
# mirrored from Ubuntu 25.04 Plucky at https://packages.ubuntu.com/plucky/kcov.
if [[ $(arch) = "x86_64" ]]; then
  dpkg_install_from_wget \
    kcov 43+dfsg-1 \
    https://drake-mirror.csail.mit.edu/ubuntu/pool/universe/k/kcov/kcov_43%2Bdfsg-1_amd64.deb \
    d192fd3cfd0d63e95f13a1b2120d0603a31b3a034b82c618d5e18205517d5cbb
fi
