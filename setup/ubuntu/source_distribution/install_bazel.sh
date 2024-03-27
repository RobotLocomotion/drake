#!/bin/bash
#
# On Ubuntu, installs either Bazel or Bazelisk at /usr/bin/bazel.
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

# Install bazel package dependencies (these may duplicate dependencies of
# drake).
apt-get install ${maybe_yes} --no-install-recommends $(cat <<EOF
g++
unzip
zlib1g-dev
EOF
)

# Install bazel.
if [[ $(arch) = "aarch64" ]]; then
  # Check if bazel is already installed.
  if [[ "$(which bazel)" ]]; then
    echo "Bazel(isk) is already installed." >&2
  else
    # TODO(jeremy.nimmer) Once there's a bazelisk 1.20 that incorporates pr563,
    # we should switch to using that here.
    dpkg_install_from_wget \
      bazelisk 1.19.0-9-g58a850f \
      https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk/pr563/bazelisk_1.19.0-9-g58a850f_arm64.deb \
      5501a44ba1f51298d186e4e66966b0556d03524381a967667696f032e292d719
  fi
else
  # Keep this version number in sync with the drake/.bazeliskrc version number.
  dpkg_install_from_wget \
    bazel 7.0.2 \
    https://github.com/bazelbuild/bazel/releases/download/7.0.2/bazel_7.0.2-linux-x86_64.deb \
    f336e7287de99e2d03953a1b2182785a94936dec6e4c1b4158457c41c509acd7
fi
