#!/bin/bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt update -y
apt install --no-install-recommends -y software-properties-common
add-apt-repository -y ppa:dreal/dreal
apt update -y

apt install -y --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
coinor-libipopt1v5
curl
libblas3
libboost-all-dev
libexpat1
libgflags-dev
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
python-lxml
python-numpy
python-scipy
python-yaml
qtbase5-dev
zlib1g
EOF
)

curl -LO https://dl.bintray.com/dreal/dreal/dreal_4.17.12.2_amd64.deb
trap 'rm -f dreal_4.17.12.2_amd64.deb' EXIT
echo '9347492e47a518ff78991e15fe9de0cff0200573091385e42940cdbf1fcf77a5  dreal_4.17.12.2_amd64.deb' | sha256sum -c
dpkg -i dreal_4.17.12.2_amd64.deb
