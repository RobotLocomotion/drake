# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "e6bb254c786ef4d67aea7e88d141b3bb2618e964",
        sha256 = "e246342c6d4e895dbc9d4d7206889114e6a6c99857987708f01066d839978beb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
