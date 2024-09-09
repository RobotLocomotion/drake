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
#
# TODO(jeremy.nimmer) Once there's a bazelisk > 1.21 that incorporates
# https://github.com/bazelbuild/bazelisk/pull/608 to publish Debian packages as
# release attachments, we should switch to official release downloads instead of
# our own build of their source release tags.
if [[ $(arch) = "aarch64" ]]; then
  dpkg_install_from_wget \
    bazelisk 1.21.0 \
    https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk/bazelisk_1.21.0_arm64.deb \
    7f75b16ac4061bc1e76e4c066ebd13ac3aba39c8e5779cd3a623ed5a66024622
else
  dpkg_install_from_wget \
    bazelisk 1.21.0 \
    https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk/bazelisk_1.21.0_amd64.deb \
    3b74a3eacbf4df24f997c6dbb23af8990e7c11907bff6cb48d0c0704a6b9be95
fi
