# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "456a2df373c4a7afeaebcd269efb9f6e86ec5579",
        sha256 = "708a1689a1911fdf0d79776cc238ac23618ce6b9eb8c4b49bd740909db82cff6",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
