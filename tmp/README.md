# Hacks for Testing latest `libsdformat` stuff

Towards testing Pose Frame Semantics out in Drake:

http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&

## Deps

For Ubuntu Bionic (may need ROS repos...)

    sudo apt install \
        libtinyxml-dev python3-vcstool python3-colcon-common-extensions

Then update repositories (and record the exact versions afterwards):

    cd drake
    mkdir -p ./tmp/repos
    vcs import ./tmp/repos < ./tmp/vcs.repo \
        && vcs export --exact ./tmp/repos > ./tmp/vcs-exact.repo

First, test without Bazel overhead:

    cd ./tmp/repos
    colcon build \
        --cmake-args \
            -DCMAKE_CXX_COMPILER=$(which g++-8)
    colcon test
    # Or just go the build package, and run `make test` / `ctest`.

Then try to build a minimal thing:

    bazel build //multibody/parsing:detail_misc

Then try other things:

    bazel test //multibody/parsing:detail_sdf_parser_test
