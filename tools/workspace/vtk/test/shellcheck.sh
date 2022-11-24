#!/bin/bash

# drake/tools/workspace/vtk/test
me="$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')"
WORKSPACE="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "${me}")")")")")"

# There must be ${WORKSPACE}/WORKSPACE.
if [ ! -f "${WORKSPACE}/WORKSPACE" ]; then
  echo "File not found: ${WORKSPACE}/WORKSPACE"
  exit 1
fi

vtk_dir="$(dirname "$(dirname "${me}")")"
# Lint the shell files associated with building the VTK tarballs.
lint_files=(
    "${me}"
    "${vtk_dir}/image/build-and-package-vtk.sh"
    "${vtk_dir}/image/clone-vtk.sh"
    "${vtk_dir}/image/common-definitions.sh"
    "${vtk_dir}/image/provision.sh"
    "${vtk_dir}/image/vtk-cmake-args.sh"
)
shellcheck -x "${lint_files[@]}"
