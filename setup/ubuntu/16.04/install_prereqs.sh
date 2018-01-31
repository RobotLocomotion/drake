#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu 16.04.

set -euo pipefail

die () {
    echo "$@" 1>&2
    trap : EXIT  # Disable line number reporting; the "$@" message is enough.
    exit 1
}

at_exit () {
    echo "${me} has experienced an error on line ${LINENO}" \
        "while running the command ${BASH_COMMAND}"
}

me='The Drake source distribution prerequisite setup script'

trap at_exit EXIT

[[ "${EUID}" -eq 0 ]] || die "${me} must run as root. Please use sudo."

apt update
apt install --no-install-recommends lsb-release

[[ "$(lsb_release -sc)" == 'xenial' ]] || die "${me} only supports Ubuntu 16.04."

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.

source "${BASH_SOURCE%/*}/install_prereqs_binary_distribution.sh"

# The following additional dependencies are only needed when developing with
# source distributions.

apt install --no-install-recommends $(tr '\n' ' ' <<EOF
bash-completion
clang-4.0
clang-format-4.0
cmake-curses-gui
coinor-libclp-dev
coinor-libipopt-dev
diffstat
doxygen
gdb
git
graphviz
kcov-34
libblas-dev
libbz2-dev
libexpat1-dev
libfreetype6-dev
libglib2.0-dev
libglu1-mesa-dev
libhdf5-dev
libjpeg8-dev
libjsoncpp-dev
liblapack-dev
liblz4-dev
libnetcdf-cxx-legacy-dev
libnetcdf-dev
libnlopt-dev
libogg-dev
libpng-dev
libqt5opengl5-dev
libqt5x11extras5-dev
libtheora-dev
libtiff5-dev
libtinyxml-dev
libtool
libxml2-dev
libxt-dev
libyaml-cpp-dev
lldb-4.0
openjdk-8-jdk
patchelf
patchutils
pkg-config
protobuf-compiler
python-dev
python-gtk2
python-matplotlib
python-protobuf
python-pygame
python-sphinx
python-tk
valgrind
zip
zlib1g-dev
EOF
)

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
    die "The ${package} deb does not have the expected SHA256.  Not installing."
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

# Install Bazel.
dpkg_install_from_wget \
  bazel 0.9.0 \
  https://github.com/bazelbuild/bazel/releases/download/0.9.0/bazel_0.9.0-linux-x86_64.deb \
  a600454ec218bffd1a1cea0f5bb511031081d23c4de15bfde674164dc2f9cd7f

# Remove a deb that we used to generate and install, but no longer need.
if [ -L /usr/lib/ccache/bazel ]; then
  apt purge ccache-bazel-wrapper
fi

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
