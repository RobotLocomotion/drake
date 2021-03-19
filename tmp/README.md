# Hacks for Testing latest `libsdformat` stuff

Towards testing Composition out in Drake:

<http://sdformat.org/tutorials?tut=composition_proposal>

## Benchmarking Models

First, make sure your git source tree isn't dirty, then run the following
script (which will do git finagling, some Bazel build stuff, etc.).

**WARNING**: This will auto-commit some artifacts.

```sh
./tmp/benchmark_against_master_and_commit_results.py
```

## Direct Rebuild of External Deps

For Ubuntu Bionic (may need ROS repos...)

```sh
sudo apt install \
    libtinyxml2-dev python3-vcstool python3-colcon-common-extensions
```

Then update repositories (and record the exact versions afterwards):

```sh
cd drake/tmp
mkdir -p ./repos
(
    set -eux
    vcs import ./repos < ./meta/vcs.repo
    vcs export --exact ./repos > ./meta/vcs-exact.repo
)
```

First, test without Bazel overhead:

```sh
cd ./tmp/repos
colcon build
colcon test
# Or just go the build package, and run `make test` / `ctest`.
```

Then update, build a minimal thing:

```sh
bazel build //multibody/parsing:detail_misc
```

Then try other things:

```sh
bazel test //multibody/parsing:detail_sdf_parser_test
```
