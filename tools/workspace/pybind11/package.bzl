# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_repository(name):
    github_archive(
        name = name,
        # TODO(eric.cousineau): Use RobotLocomotion.
        repository = "EricCousineau-TRI/pybind11",
        commit = "cb55345e012314f0b1632ed4c4717d8dd2df71ba",
        sha256 = "390d81c3f8967872080c0cb1269861e2f714dc2a5caf0dd222f60811c59a1a2f",  # noqa
        build_file = "@drake//tools/workspace/pybind11:package.BUILD.bazel",
    )
