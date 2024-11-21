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
arch="$(/usr/bin/arch)"
clang_version="$(clang --version |head -n1 |sed 's#.*(clang-\([0-9]*\)\..*#\1#g')"

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/macos.bazelrc
import %workspace%/tools/macos-arch-${arch}.bazelrc
EOF

if (( $clang_version >= 1600 )); then
cat >> "${bazelrc}" <<EOF
import %workspace%/tools/apple-clang-1600.bazelrc
EOF
fi

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
(cd "${workspace_dir}" && bazelisk version) > /dev/null
