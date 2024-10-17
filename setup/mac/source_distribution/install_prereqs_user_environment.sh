#!/bin/bash
#
# Writes user environment prerequisites for source distributions of Drake on
# macOS.

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bazelrc"
arch="$(/usr/bin/arch)"

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/macos.bazelrc
import %workspace%/tools/macos-arch-${arch}.bazelrc
EOF

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
(cd "${workspace_dir}" && bazelisk version) > /dev/null
