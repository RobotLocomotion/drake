#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

[ -d /tmp/drake-wheel-build/ ]

mkdir /tmp/drake-wheel-build/drake-build
cd /tmp/drake-wheel-build/drake-build

# Store downloads in the build cache to speed up rebuilds.
export BAZELISK_HOME=/var/cache/bazel/bazelisk

# Add wheel-specific settings that are not available via CMake options.
# N.B. When you change anything here, also fix wheel/macos/build-wheel.sh.
cat > /tmp/drake-wheel-build/drake-build/drake.bazelrc << EOF
build --disk_cache=/var/cache/bazel/disk_cache
build --repository_cache=/var/cache/bazel/repository_cache
build --repo_env=DRAKE_WHEEL=1
# Enable MOSEK lazy loading. Right now this is only done for Linux builds.
build --@drake//solvers:mosek_lazy_load=True
EOF

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
PYTHON_MINOR=$(/usr/local/bin/python -c "import sys; print(sys.version_info.minor)")
WITH_MOSEK=ON
[ ${PYTHON_MINOR} -ge 15 ] && WITH_MOSEK=OFF

# MOSEK is not currently supported for Linux aarch64 wheels.
[ "$(arch)" == "aarch64" ] && WITH_MOSEK=OFF

# Install Drake using our wheel-build-specific Python interpreter and
# C/CXX/Fortran compilers.
# N.B. When you change anything here, also fix wheel/macos/build-wheel.sh.
cmake ../drake-src \
    -DWITH_OPENMP=ON \
    -DWITH_MOSEK="${WITH_MOSEK}" \
    -DWITH_SNOPT=ON \
    -DSNOPT_PATH="${SNOPT_PATH}" \
    -DWITH_USER_EIGEN=OFF \
    -DWITH_USER_FMT=OFF \
    -DWITH_USER_SPDLOG=OFF \
    -DWITH_USER_BLAS=OFF \
    -DWITH_USER_LAPACK=OFF \
    -DWITH_USER_ZLIB=OFF \
    -DDRAKE_INSTALL_JAVA=OFF \
    -DDRAKE_VERSION_OVERRIDE="${DRAKE_VERSION}" \
    -DDRAKE_GIT_SHA_OVERRIDE="${DRAKE_GIT_SHA}" \
    -DCMAKE_INSTALL_PREFIX=/tmp/drake-wheel-build/drake-dist \
    -DPython_EXECUTABLE=/usr/local/bin/python \
    -DCMAKE_C_COMPILER=/opt/rh/gcc-toolset-14/root/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/opt/rh/gcc-toolset-14/root/usr/bin/g++ \
    -DCMAKE_Fortran_COMPILER=/opt/rh/gcc-toolset-14/root/usr/bin/gfortran
make install
