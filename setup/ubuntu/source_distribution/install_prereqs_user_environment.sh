#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# Ubuntu.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bzl"
codename=$(lsb_release -sc)

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/ubuntu-${codename}.bazelrc
EOF
