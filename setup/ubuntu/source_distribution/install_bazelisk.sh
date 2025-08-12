#!/bin/bash
#
# On Ubuntu, installs bazelisk at /usr/bin/bazel{,isk}.
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

# If bazel.deb is already installed, we'll need to remove it first because
# the Debian package of bazelisk will take over the `/usr/bin/bazel` path.
apt-get remove bazel || true

# Install bazelisk.
if [[ $(arch) = "aarch64" ]]; then
  dpkg_install_from_wget \
    bazelisk 1.27.0 \
    https://github.com/bazelbuild/bazelisk/releases/download/v1.27.0/bazelisk-arm64.deb \
    173c5b367b485a30ce58c1d0d560b39d257a2d7a3c859c45d7d05eb61605a2a1
else
  dpkg_install_from_wget \
    bazelisk 1.27.0 \
    https://github.com/bazelbuild/bazelisk/releases/download/v1.27.0/bazelisk-amd64.deb \
    d8b00ea975c823e15263c80200ac42979e17368547fbff4ab177af035badfa83
fi
