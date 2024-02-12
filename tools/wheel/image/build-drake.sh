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
build --repo_env=DRAKE_OS=manylinux
build --repo_env=SNOPT_PATH=git
build --config=packaging
build --define=LCM_INSTALL_JAVA=OFF
# Our wheel Eigen is new enough to undo the Focal-specific removal of Clarabel.
build --define=NO_CLARABEL=OFF
EOF

# Install Drake using our wheel-build-specific Python interpreter.
cmake ../drake \
    -DCMAKE_INSTALL_PREFIX=/opt/drake \
    -DPython_EXECUTABLE=/usr/local/bin/python
make install
