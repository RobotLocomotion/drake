#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

mkdir /opt/drake-wheel-build/drake-build
cd /opt/drake-wheel-build/drake-build

# Store downloads in the build cache to speed up rebuilds.
export BAZELISK_HOME=/var/cache/bazel/bazelisk

# Add wheel-specific bazel options.
cat > /opt/drake-wheel-build/drake-build/drake.bazelrc << EOF
build --disk_cache=/var/cache/bazel/disk_cache
build --repository_cache=/var/cache/bazel/repository_cache
build --repo_env=DRAKE_WHEEL=1
build --repo_env=SNOPT_PATH=${SNOPT_PATH}
build --config=packaging
build --define=LCM_INSTALL_JAVA=OFF
build --@@spdlog+//:header_only=False
EOF

# Install Drake using our wheel-build-specific Python interpreter.
cmake ../drake \
    -DWITH_USER_EIGEN=OFF \
    -DWITH_USER_FMT=OFF \
    -DWITH_USER_SPDLOG=OFF \
    -DDRAKE_VERSION_OVERRIDE="${DRAKE_VERSION}" \
    -DDRAKE_GIT_SHA_OVERRIDE="${DRAKE_GIT_SHA}" \
    -DCMAKE_INSTALL_PREFIX=/opt/drake \
    -DPython_EXECUTABLE=/usr/local/bin/python
make install
