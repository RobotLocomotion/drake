# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/pybind11",
        commit = "48999b69bde29cdf8d616d4fbd3d6ab1c561027d",
        sha256 = "2ea18adfb608948cab1b5978081dc8c318ed47573ccd66f1603a37fbdbfc56da",  # noqa
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )
