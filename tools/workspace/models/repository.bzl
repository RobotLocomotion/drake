# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5a4ae63c6a1922c9b3414a426c3568ae96aa0b1d",
        sha256 = "81c1c282a74701bb26967830ffdb7d8a2bdbfae7b35acdae48d5bf380a4ae679",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
