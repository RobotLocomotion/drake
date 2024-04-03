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
import %workspace%/tools/ubuntu.bazelrc
import %workspace%/tools/ubuntu-${codename}.bazelrc
EOF

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
if [[ $(arch) = "aarch64" ]]; then
  # Per ./install_bazel.sh, on arm we use bazelisk as our bazel so we should
  # prefetch using that spelling.
  cd "${workspace_dir}" && bazel version
else
  # Per ./install_bazel.sh, on non-arm there is no system-wide bazelisk so we
  # need to use a local copy.
  cd "${workspace_dir}" && ./third_party/com_github_bazelbuild_bazelisk/bazelisk.py version
fi
