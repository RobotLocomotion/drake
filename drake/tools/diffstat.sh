#!/bin/bash

# diffstat.sh -- Report lines of code added or changed in the current tree,
# versus upsteam/master, excluding files that do not need platform review.

set -e

# This is what reviewable.io looks for to indicate generated code.
# See https://github.com/Reviewable/Reviewable/wiki/FAQ.
marker="GENERATED FILE ""DO NOT EDIT"

# Exclude files from /dev/, and then also exclude files with the $marker.
excludes=--exclude="*/dev/*"
toplevel=$(git rev-parse --show-toplevel)
git_base_path=upstream/master
for relpath in $(git diff $git_base_path | lsdiff $excludes --strip=1); do
    if fgrep -q -e"$marker" $toplevel/$relpath; then
        excludes="$excludes --exclude=$relpath"
    fi
done

# Display a final summary.
git diff $git_base_path |
    filterdiff --strip-match=1 --strip=1 $(echo $excludes) |
    diffstat -p 0
