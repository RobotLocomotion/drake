#!/bin/bash

# Checks that libdrake.so doesn't contain a build of spdlog.

set -eu -o pipefail

cd "$(mktemp -d)"

root=/tmp/drake-wheel-test/python/lib/python*/site-packages/pydrake
libdrake=$root/lib/libdrake.so

# Install binutils.
. /etc/os-release
case "$ID" in
  "ubuntu")
    apt-get install -y binutils
    ;;
  "amzn")
    dnf install -y binutils
    ;;
  *)
esac

# Run the check.
nm -D --defined-only $libdrake > out
if grep -q -e "N6spdlog" out; then
  echo "ERROR: 'spdlog' symbols exist in libdrake."
  exit 1
fi
