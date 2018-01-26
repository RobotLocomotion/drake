#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# macOS.

set -euo pipefail

# We require that Bazel uses the Python installed by Homebrew.
# TODO(jamiesnape): Also support a .bazelrc located in the WORKSPACE.
if [[ ! -f "${HOME}/.bazelrc" ]] || ! grep -q '^build --python_path=' "${HOME}/.bazelrc"; then
  echo "We need to add 'build --python_path=/usr/local/opt/python/libexec/bin/python' to ~/.bazelrc."
  read -r -p 'Do you want to continue (y/N)? ' reply
  if [[ "${reply}" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
    echo 'build --python_path=/usr/local/opt/python/libexec/bin/python' >> "${HOME}/.bazelrc"
  fi
fi

if [[ ! -f "${HOME}/.bazelrc" ]] || ! grep -q '^build --python_path=/usr/local/opt/python/libexec/bin/python$' "${HOME}/.bazelrc"; then
  echo 'Using a python other than /usr/local/opt/python/libexec/bin/python is NOT supported' >&2
fi
