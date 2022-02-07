# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f0b1c84aa00ab4ce4b2e0c4ef60b61fb4f6dd2bf",
        sha256 = "af896ad6a1ebf1fdefeb63ab73b022344e26df51cee28ad654f7946dc69498d3",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
