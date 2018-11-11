#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu 18.04 (Bionic). Note that this is currently an UNSUPPORTED platform.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt-get install --no-install-recommends $(tr '\n' ' ' <<EOF
apt-transport-https
ca-certificates
gnupg
wget
EOF
)

wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add
echo 'deb [arch=amd64] https://drake-apt.csail.mit.edu/bionic bionic main' > /etc/apt/sources.list.d/drake.list

apt-get update
apt-get install --no-install-recommends $(cat "${BASH_SOURCE%/*}/packages.txt" | tr '\n' ' ')

locale-gen en_US.UTF-8

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
    echo "The ${package} deb does not have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

dpkg_install_from_wget \
  bazel 0.18.0 \
  https://github.com/bazelbuild/bazel/releases/download/0.18.0/bazel_0.18.0-linux-x86_64.deb \
  fbdc6dd3bbac4512648314619a317de81df6bbea9826d9201c4e07ec48d3744f
