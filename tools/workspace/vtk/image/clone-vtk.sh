# shellcheck shell=bash
# This file is *sourced* from both the docker and macOS builds for VTK to define
# common variables such as what to clone and the output archive naming scheme.
# This script must be compatible with bash 3+ (due to macOS).
#
# 1. This script assumes the calling script's current working directory is ready
#    for a `git clone`, VTK is cloned into the `src` directory.
# 2. This script populates the variable `vtk_archive_name` variable used in the
#    wrapping build scripts for the final `.tar.gz` archive name.
#
# The script defines for the sourcing parent script `vtk_archive_name` which
# has the full name of the final .tar.gz archive to be used, including the
# `git describe`, platform, architecture, build number, etc.
if [[ "${0}" == "${BASH_SOURCE[0]}" ]]; then
    echo 'clone-vtk.sh: this file must be sourced.' >&2
    exit 1
fi
# shellcheck disable=SC2154
if [[ -z "${os_name}" ]] || [[ -z "${architecture}" ]] || \
   [[ -z "${codename}" ]]; then
    echo "clone-vtk.sh: the 'architecture', 'codename', and 'os_name' " >&2
    echo "variables were expected to be set, but had no value.  Source " >&2
    echo "common-definitions.sh." >&2
    exit 1
fi

# Anytime `vtk_git_ref` is updated, the `build_number` variables defined below
# should all be reset to 1.
readonly vtk_git_ref="d706250a1422ae1e7ece0fa09a510186769a5fec"

git clone https://gitlab.kitware.com/vtk/vtk.git src
git -C src checkout "${vtk_git_ref}"

# Will include the latest tag and additional commit information if the
# vtk_git_ref does not point to a tag, e.g., v9.2.2-1383-g7798461b95.
readonly vtk_version="$(git -C src describe)"

# For creating the archive names, we want to include:
#
#     vtk-${vtk_version}-${codename}-${architecture}-${build_number}.tar.gz
#
# where `build_number` is included in the event that an artifact needs to be
# repackaged without the VTK version changing.  Never overwrite an artifact on
# drake-packages s3 bucket, you will break historical builds.
# Define build numbers separately for maintenance convenience.
if [[ "${os_name}" == "Darwin" ]]; then
    if [[ "${architecture}" == "arm64" ]]; then
        readonly build_number="1"  # macOS arm64
    else
        readonly build_number="1"  # macOS x86_64
    fi
else
    if [[ "${architecture}" == "arm64" ]]; then
        readonly build_number="1"  # Linux arm64 (NOTE: currently unused)
    else
        readonly build_number="1"  # Linux x86_64
    fi
fi

# The final archive output name used by the build scripts.
# shellcheck disable=SC2034
readonly vtk_archive_name="vtk-${vtk_version}-${codename}-${architecture}-${build_number}.tar.gz"
