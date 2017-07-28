#!/bin/bash
#
# Prerequisite set-up script for a Drake build with Bazel on Ubuntu 14.04.

set -euo pipefail

apt update
apt install --no-install-recommends $(tr "\n" " " <<EOF
curl
lsb-release
software-properties-common
EOF
)

# Add APT repository for (clang|clang-format|lldb)-3.9.
readonly lsb_release_codename="$(lsb_release -sc)"
# In this form, add-apt-repository is only truly idempotent when --enable-source
# is added, since it otherwise duplicates the commented deb-src line.
add-apt-repository --enable-source \
    "deb http://apt.llvm.org/${lsb_release_codename}/ llvm-toolchain-${lsb_release_codename}-3.9 main"
curl --location http://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add -

# Add APT repository for (g++|gcc|gfortran)-(4.9|5).
add-apt-repository ppa:ubuntu-toolchain-r/test

# Add APT repository for oracle-java8-installer.
add-apt-repository ppa:webupd8team/java

apt update
apt install --no-install-recommends $(tr "\n" " " <<EOF
bash-completion
binutils
clang-3.9
clang-format-3.9
doxygen
g++
g++-4.9
g++-4.9-multilib
gcc
gcc-4.9
gcc-4.9-multilib
gdb
gfortran
gfortran-4.9
gfortran-4.9-multilib
git
graphviz
libboost-dev
libexpat1
libfreetype6
libglib2.0-dev
libglu1-mesa-dev
libjpeg8
libpng-dev
libqt4-opengl
libtiff5
libtinyxml-dev
libtool
libxml2
libxt6
lldb-3.9
make
mesa-common-dev
oracle-java8-installer
patchutils
pkg-config
python-dev
python-numpy
python-sphinx
valgrind
zip
zlib1g-dev
EOF
)

# Download, verify, and install Bazel 0.5.2.
pushd /tmp
readonly bazel_version="0.5.2"
readonly bazel_deb_filename="bazel_${bazel_version}-linux-x86_64.deb"
curl --location --remote-name \
    "https://github.com/bazelbuild/bazel/releases/download/${bazel_version}/${bazel_deb_filename}"
echo "b14c8773dab078d3422fe4082f3ab4d9e14f02313c3b3eb4b5b40c44ce29ed59 ${bazel_deb_filename}" \
    > "${bazel_deb_filename}.sha256"
sha256sum --check "${bazel_deb_filename}.sha256" \
    && dpkg --install "${bazel_deb_filename}"
rm "${bazel_deb_filename}.sha256" "${bazel_deb_filename}"
popd
