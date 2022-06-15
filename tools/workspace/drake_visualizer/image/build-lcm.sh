#!/bin/bash

set -eu -o pipefail

# Note that we use LCM and its dependency JChart2D from Drake, partly due to
# mangled name of the LCM library in Drake. We do not use JChart2D, but
# "find_package(LCM)" would fail without it. LCM is LGPL 2.1 so we
# should not link it statically like we do for the other C/C++ dependencies of
# drake-visualizer.
#
# TODO(jamiesnape): Maybe we should bundle a separate version anyway, but are
# different LCM versions always compatible with Drake?
#
# Note that the added options are to follow the Debian build hardening
# guidelines, which is good practice anyway.
git clone --depth 1 \
    https://github.com/RobotLocomotion/drake.git /drake

cd /drake
# Drake requires the setup script to be installed in order to load
# drake/gen/environment.bazelrc, this command generates this file.
./setup/ubuntu/source_distribution/install_prereqs_user_environment.sh

# Patch command is required to build lcm.
apt-get install -y --no-install-recommends patch

bazel run \
    --copt=-fstack-protector-strong \
    --host_copt=-fstack-protector-strong \
    --linkopt=-Wl,-Bsymbolic-functions \
    --linkopt=-Wl,-z,now \
    --linkopt=-Wl,-z,relro \
    @lcm//:install -- /opt/drake
