#!/bin/bash
#
# Install Bazel as /usr/bin/bazel on Ubuntu 20.04 (Focal) or 22.04 (Jammy).
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
# Keep this version number in sync with the drake/.bazeliskrc version number.
if [[ $(arch) = "aarch64" ]]; then
  # Check if bazel is already installed.
  if [[ "$(which bazel)" ]]; then
    echo "Bazel is already installed." >&2
  else
    echo "WARNING: On Ubuntu arm64 systems, Drake's install_prereqs does not" \
    "automatically install Bazel on your behalf. You will need to install" \
    "Bazel yourself. See https://bazel.build for instructions." >&2
  fi
else
  dpkg_install_from_wget \
    bazel 6.4.0 \
    https://github.com/bazelbuild/bazel/releases/download/6.4.0/bazel_6.4.0-linux-x86_64.deb \
    9276a1e11f03e9f7492f009803c95bddc307993c9ab3c463721c9f6cdaa2ccc1
fi
