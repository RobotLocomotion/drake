# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/pybind11",
        commit = "a7e9d1ba30703dbdd97eefd45c314b91fc0eccd8",
        sha256 = "a4d0fcbaaf837b1038d6a5df83a9377c61c932bb8b470fcf881066bf6ca4eee3",  # noqa
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )
