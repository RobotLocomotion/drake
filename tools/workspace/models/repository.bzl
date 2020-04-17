# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "3a4627b5207de76c2f86a13f522da87cac882730",
        sha256 = "c6d02c310a47922ac5b4f9ddbe6a800e13cd8e37b534c73bc5e1b216261d36ba",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
