#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# Ubuntu.

set -euo pipefail

prefetch_bazel=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Prefetch bazel, if installing bazel.
    --prefetch-bazel)
      prefetch_bazel=1
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 3
  esac
  shift
done

# Check for existence of `SUDO_USER` so that this may be used in Docker
# environments.
if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:+D}" ]]; then
  echo 'This script must NOT be run through sudo as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bazelrc"
arch=$(/usr/bin/arch)
codename=$(lsb_release -sc)

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/ubuntu.bazelrc
import %workspace%/tools/ubuntu-arch-${arch}.bazelrc
import %workspace%/tools/ubuntu-${codename}.bazelrc
EOF

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
if [[ "${prefetch_bazel}" -eq 1 ]]; then
  (cd "${workspace_dir}" && bazel version)
fi

# Our MODULE.bazel uses this file to determine the default python version.
/usr/bin/python3 -c "from sys import version_info as v; print('{}.{}'.format(v.major, v.minor))" > "${workspace_dir}/gen/python_version.txt"
