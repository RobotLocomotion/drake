# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        # Working temporary SHA1 until the models PR 9 is merged.
        commit = "5e8df93644343130eb25115305340e10cf8f0c72",
        sha256 = "4dde0fabfaa0621c715b86003c8c5357a7557ce5e9bceb67bea69cd0dccec155",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
