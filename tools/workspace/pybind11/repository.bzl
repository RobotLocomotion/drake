# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/pybind11",
        commit = "060f8eb02c72c793967ccdaa6da10d6aba3055db",
        sha256 = "5a250d01cb4ace5d7e15bdd5b71c4e8f041bc7965677384903aeb77898953602",  # noqa
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )
