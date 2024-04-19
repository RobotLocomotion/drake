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
# TODO(jeremy.nimmer) Once there's a bazelisk >= 1.20 that incorporates
# https://github.com/bazelbuild/bazelisk/pull/563, we should switch to
# official release downloads instead of our Drake-custom Debian packages.
if [[ $(arch) = "aarch64" ]]; then
  dpkg_install_from_wget \
    bazelisk 1.19.0-9-g58a850f \
    https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk/pr563/bazelisk_1.19.0-9-g58a850f_arm64.deb \
    5501a44ba1f51298d186e4e66966b0556d03524381a967667696f032e292d719
else
  dpkg_install_from_wget \
    bazelisk 1.19.0-9-g58a850f \
    https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk/pr563/bazelisk_1.19.0-9-g58a850f_amd64.deb \
    c2bfd15d6c3422ae540cda9facc0ac395005e2701c09dbb15d40447b53e831d4
fi
