#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# Ubuntu.

set -euo pipefail

# Check for existence of `SUDO_USER` so that this may be used in Docker
# environments.
if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:+D}" ]]; then
  echo 'This script must NOT be run through sudo as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bazelrc"
codename=$(lsb_release -sc)

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/ubuntu-${codename}.bazelrc
EOF
