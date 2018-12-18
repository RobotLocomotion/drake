#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# Ubuntu.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
gen_dir="${workspace}/gen"
mkdir -p "${gen_dir}"

codename=$(lsb_release -sc)
echo > "${gen_dir}/environment.bzl" <<EOF
import %workspace%/tools/ubuntu-${codename}.bazelrc
EOF
