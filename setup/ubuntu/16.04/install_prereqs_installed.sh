#!/bin/bash

# Copyright (c) 2018, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the Massachusetts Institute of Technology nor the names
#   of its contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE MASSACHUSETTS INSTITUTE OF TECHNOLOGY AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE MASSACHUSETTS
# INSTITUTE OF TECHNOLOGY OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt update -y
apt install --no-install-recommends -y software-properties-common curl
add-apt-repository -y ppa:dreal/dreal
apt update -y

apt install -y --no-install-recommends $(tr '\n' ' ' <<EOF
bison
build-essential
cmake
coinor-libipopt1v5
flex
libblas3
libboost-all-dev
libexpat1
libgflags2v5
libgl1-mesa-glx
libglib2.0-0
libhdf5-10
libibex-dev
libjpeg8
libjsoncpp1
liblapack3
libnetcdf11
libnetcdf-c++4
libnlopt0
libogg0
libpng12-0
libprotobuf-dev
libqt5multimedia5
libqt5x11extras5
libtheora0
libtiff5
libtinyxml2-dev
libxml2
libxt6
libyaml-cpp0.5v5
openjdk-8-jre
pkg-config
python-gtk2
python-lxml
python-numpy
python-scipy
python-yaml
qtbase5-dev
EOF
)

curl -LO https://dl.bintray.com/dreal/dreal/dreal_4.17.12.2_amd64.deb
trap 'rm -f dreal_4.17.12.2_amd64.deb' EXIT
echo '9347492e47a518ff78991e15fe9de0cff0200573091385e42940cdbf1fcf77a5  dreal_4.17.12.2_amd64.deb' | sha256sum -c
dpkg -i dreal_4.17.12.2_amd64.deb
