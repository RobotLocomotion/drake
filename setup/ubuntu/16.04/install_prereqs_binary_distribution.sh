#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 16.04.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt update
apt install --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
coinor-libipopt1v5
libblas3
libboost-all-dev
libexpat1
libgflags-dev
libgl1-mesa-glx
libglib2.0-0
libhdf5-10
libjpeg8
libjsoncpp1
liblapack3
libnetcdf-c++4
libnetcdf11
libnlopt0
libogg0
libpng12-0
libprotobuf-dev
libqt5multimedia5
libqt5x11extras5
libtheora0
libtiff5
libtinyxml2-dev
libtinyxml2.6.2v5
libxml2
libxt6
libyaml-cpp0.5v5
openjdk-8-jre
python-lxml
python-numpy
python-scipy
python-yaml
qtbase5-dev
wget
zlib1g
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

# TODO(m-chaturvedi): Remove the following apt install call when the
# dependencies of the dreal package are corrected.
apt install --no-install-recommends $(tr '\n' ' ' <<EOF
bison
coinor-libclp-dev
flex
libbz2-dev
libnlopt-dev
pkg-config
zlib1g-dev
EOF
)

# Install IBEX, a dReal dependency.  See
# https://launchpad.net/~dreal/+archive/ubuntu/dreal
# for more information. To rebuild IBEX, add the PPA `ppa:dreal/dreal` and then
# run `apt source libibex-dev` to get the sources.
dpkg_install_from_wget \
  libibex-dev 2.6.5.20180123154310.gitf618c7b296182f90a84d54936d144b87df0747b9~16.04 \
  https://dl.bintray.com/dreal/ibex/libibex-dev_2.6.5_amd64.deb \
  5519f6e3ec53f92dcd4c461dfb599b11d1973a57638646d42612c3cb741679dc

# Install dReal. See
# https://github.com/dreal/dreal4/blob/master/README.md#build-debian-package for
# build instructions.
dpkg_install_from_wget \
  dreal 4.18.01.3 \
  https://dl.bintray.com/dreal/dreal/dreal_4.18.01.3_amd64.deb \
  dcac76d7ba183014d9db7c5d1a5a0960e2a744e11769853fde83f03af052459b
