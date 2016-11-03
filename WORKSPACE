# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake")

new_http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "tools/gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0",
)

new_git_repository(
    name = "eigen",
    remote = "https://github.com/RobotLocomotion/eigen-mirror.git",
    commit = "1b7acef29a4c53b5867e5d9da7e97bde436219f9",
    build_file = "tools/eigen.BUILD",
)
