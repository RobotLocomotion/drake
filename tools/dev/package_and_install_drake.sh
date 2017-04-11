#!/bin/bash
### Wraps drake/tools/package_drake.sh such that it (a) provides artifacts in 
### BUILD_DIR and INSTALL_DIR, and (b) will incrementally update
### the artifacts in INSTALL_DIR based on what is present in BUILD_DIR, and
### (c) it will only update the artifacts in BUILD_DIR if there is a detected
### change in the Drake git repository (a dirty repository means it will always
### regenerate the build artifacts).
###
### Motivation: Alleviate the fact that Bazel does not readily provide an
### incremental installation mechanism to aid downstream users in their
### development process.
###
### Usage:
###     package_and_install_drake.sh <BUILD_DIR> <INSTALL_DIR>
### 
### @param BUILD_DIR The build / staging directory to place the artifacts to
### be installed. This will be created if it does not exist.
### You may use $(mktemp -d) if you do not need incremental 
### build; however, in this case, you should just use package_drake.sh 
### directly.
### @param INSTALL_DIR The directory to which artifacts will be incrementally
### installed (i.e., their timestamp will only change if their content has
### changed). This will be created if it does not exist.
###
### Example:
###   $ ./package_and_install_drake.sh /tmp/build /tmp/build/install
set -e -u

# This uses git to incrementally change the timestamps, due to its
# effectiveness and portability.
# 
# The following alternatives were considered:
# 
# * install - This does not preserve the timestamps, even with `--compare` and
# `--preserve-timestamps` (which only preserves copying the source timestamp
# to a destination file). Additionally, these are GNU options, and other
# issues may be encountered with BSD tools on Mac OSX.
#     * Attempt: https://git.io/v9eNf
# 
# * rsync - This also does not preserve timestamps.
#     * Attempt: https://git.io/v9eNI
# 
# These alternatives were tested with this comparison script:
# 
# * https://git.io/v9ebp - Modifies timestamps in INSTALL_DIR directory to be
# to be older than BUILD_DIR, and then prints the timestamps after install.
# This test script tests the three methods: (1) git, (2) install, and
# (3) rsync.
# This test script should be executed twice, first to get the baseline content
# (INSTALL_DIR timestamps will new), and second to check the timestamp after
# (INSTALL_DIR timestamps should be older than BUILD_DIR's).
# 
# * https://git.io/v9ebN - Example output from running

usage() {
    echo "Usage: $(basename $0) <BUILD_DIR> <INSTALL_DIR>"
}
error() {
    echo "$@" >&2
}

[[ $# -eq 2 ]] || { error "Incorrect arguments specified"; usage; exit 1; }

mkcd() {
    mkdir -p "$1" && cd "$1";
}

drake=$(cd $(dirname $BASH_SOURCE) && git rev-parse --show-toplevel)
build_dir=$(mkcd $1 && pwd)
install_dir=$(mkcd $2 && pwd)

# TODO(eric.cousineau) Consider a less hacky-method to ensure that CMake's
# find_package will search PATH, detect ./bin/, and resolve to parent
# directory to search in lib/cmake/
# @ref https://cmake.org/cmake/help/v3.0/command/find_package.html
mkdir -p $install_dir/bin

drake_build=$build_dir/drake
mkdir -p $drake_build

# Use git-checkout to handle hashing artifacts and check if things
# need to be updated
# Will use one repository for staging, and then a second for "deploying"
# with minimal timestamp changes
drake_package_git=$drake_build/package-git
[[ -d $drake_package_git ]] || (
        mkcd $drake_package_git;
        git init --quiet .;
        # Ignore nothing, override user ~/.gitignore
        git config core.excludesfile ''
    )

# Only rebuild if either (a) $drake git was previously or currently dirty, or
# (b) if the SHAs in $drake git do not match those in $drake_git_status_file
# stored in BUILD_DIR
git_ref() {
    # http://stackoverflow.com/a/5737794/7829525
    if [[ -n "$(git status --porcelain)" ]]; then
        echo "dirty"
    else
        git rev-parse --short HEAD
    fi
}
drake_git_status_file=$drake_build/drake-git-status
cur_git_status=$(cd $drake && git_ref)
need_rebuild=1
if [[ -e $drake_git_status_file ]]; then
    prev_git_status=$(cat $drake_git_status_file)
    echo "Prior build exists ($prev_git_status)."
    echo "  Check against current ($cur_git_status)"
    if [[ -z $cur_git_status || -z $prev_git_status \
            || $cur_git_status = "dirty" || $prev_git_status = "dirty" ]]; then
        echo "Rebuild needed: current or previous build was dirty"
    elif [[ $cur_git_status != $prev_git_status ]]; then
        echo "Rebuild needed: current and previous build on different commits"
    else
        echo "No rebuild needed"
        need_rebuild=
    fi
else
    echo "First build"
fi

if [[ -z "$need_rebuild" ]]; then
    exit 0;
fi

echo "Generate package artifact from //drake/tools"
package=$drake_build/package.tar.gz
$drake/tools/package_drake.sh $package

cd $drake_package_git
# Remove prior files
rm -rf ./*

echo "Extract and commit current version"
tar xfz $package
git add -A :/
git commit --quiet -m \
    "Package artifacts for drake (status: $cur_git_status)" || {
        echo "No artifact difference detected." \
            "Skipping the rest of the rebuild.";
        echo "$cur_git_status" > $drake_git_status_file;
        exit 0;
    }

echo "Checkout and allow Git to handle deltas"
echo "  Be conservative on changing timestamps."
git --work-tree=$install_dir checkout --quiet -f HEAD -- .

# TODO(eric.cousineau): If space usage is an issue, then the following can be
# done:
# 
# 1. Delete existing .git/ directory
# 2. Reinitialize, and recommit
# 
# This will get rid of unused history.

# On success, dump the git status
echo "$cur_git_status" > $drake_git_status_file
echo "Done"
