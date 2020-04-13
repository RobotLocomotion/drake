# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "896aab19f7c0eaa1e85241ab1caf565bbb70b092",
        sha256 = "821d762240bd20673f3ef48d84b4ea01a206de33a758792a3d6ccea6c81a7882",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
