# Hacks for Testing latest `libsdformat` stuff

Towards testing Pose Frame Semantics out in Drake:

http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&

## Deps

For Ubuntu Bionic (may need ROS repos...)

    sudo apt install \
        libtinyxml-dev python3-vcstool python3-colcon-common-extensions

Then update repositories:

    cd drake/tmp
    mkdir -p repos
    vcs import ./repos < vcs.repo

First, test without Bazel overhead:

    cd drake/tmp
    colcon build

Then try to build a minimal thing:

    bazel build //multibody/parsing:detail_misc
