# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "745547fbec8ec8cb10be6ef23f7f0cd5ba49d321",
        sha256 = "a7ab36339c935a6e487f0275d8c5a38148658641e05207201cfd591d2ce60437",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
