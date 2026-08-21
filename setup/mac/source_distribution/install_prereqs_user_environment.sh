#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# macOS.

# N.B. Ensure that this is synchronized with the install instructions regarding
# Homebrew Python in `doc/python_bindings.rst`.

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bazelrc"

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/macos.bazelrc
EOF

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
(cd "${workspace_dir}" && bazelisk version) > /dev/null

# Our MODULE.bazel uses this file to determine the default python version.
# When changing this, see drake/tools/workspace/python/README.md.
echo "3.14" > "${workspace_dir}/gen/python_version.txt"
