#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# macOS.

# N.B. Ensure that this is synchronized with the install instructions regarding
# Homebrew Python in `doc/python_bindings.rst`.

set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bzl"

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
import %workspace%/tools/macos.bazelrc
EOF

if [[ -f "${HOME}/.bazelrc" ]] && /usr/bin/grep -q '^build --python_path=' "${HOME}/.bazelrc"; then
  cat >&2 <<EOF
WARNING: Python paths are now specified per workspace for Drake.
  Please remove lines from ~/.bazelrc that change --python_path so that there
  are no conflicts with the project configuration.
EOF
fi
