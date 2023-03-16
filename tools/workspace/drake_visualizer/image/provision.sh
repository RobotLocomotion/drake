#!/bin/bash

set -eu -o pipefail

export DEBIAN_FRONTEND=noninteractive

readonly BAZEL_VERSION=6.1.1
readonly BAZEL_ROOT=https://github.com/bazelbuild/bazel/releases/download

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

xargs -d$'\n' apt-get -y install --no-install-recommends < /image/prereqs

# Install Bazel.
apt-get -y install --no-install-recommends unzip wget
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
