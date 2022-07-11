# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "fbc442f5bb609ecd8c32d8ab6e033b998a929692",
        sha256 = "104662b08af4e2681cefa309de5c47c25a6b82531a1cda5f8e554c7b8c296831",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
