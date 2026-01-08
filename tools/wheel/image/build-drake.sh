#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

[ -d /tmp/drake-wheel-build/ ]

mkdir /tmp/drake-wheel-build/drake-build
cd /tmp/drake-wheel-build/drake-build

# Store downloads in the build cache to speed up rebuilds.
export BAZELISK_HOME=/var/cache/bazel/bazelisk

# Add wheel-specific bazel options.
# N.B. When you change anything here, also fix wheel/macos/build-wheel.sh.
cat > /tmp/drake-wheel-build/drake-build/drake.bazelrc << EOF
build --disk_cache=/var/cache/bazel/disk_cache
build --repository_cache=/var/cache/bazel/repository_cache
build --repo_env=DRAKE_WHEEL=1
build --repo_env=SNOPT_PATH=${SNOPT_PATH}
build --config=packaging
# Enable MOSEK lazy loading. Right now this is only done for Linux builds.
build --@drake//solvers:mosek_lazy_load=True
EOF

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
PYTHON_MINOR=$(/usr/local/bin/python -c "import sys; print(sys.version_info.minor)")
if [[ ${PYTHON_MINOR} -ge 15 ]]; then
    cat >> /tmp/drake-wheel-build/drake-build/drake.bazelrc << EOF
build --@drake//tools/flags:with_mosek=False
build --@drake//solvers:mosek_lazy_load=False
EOF
fi

# Install Drake using our wheel-build-specific Python interpreter.
# N.B. When you change anything here, also fix wheel/macos/build-wheel.sh.
cmake ../drake-src \
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
    -DPython_EXECUTABLE=/usr/local/bin/python
make install
