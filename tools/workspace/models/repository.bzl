# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "74a04a817842bfe7bb32fc563f04c37f3eba4e9b",
        sha256 = "3636c5813e96d4e2a291e35414e570f36c4aa3c2af772f089ff07d807da0b9ea",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
