# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "0fd6eb4b3bba6dd044ef43367cba38675483678f",
        sha256 = "c0dcc94996da39d212cdaa7763130954551c6e9f86e73ff7751232b836c05875",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
