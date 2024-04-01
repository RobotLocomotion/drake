load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "b2f1a5de046d0b0d27dd5b8f09020addc9a19280",
        sha256 = "1d8cf6d08e317a9f011f30e28669e8eede8cffe249a4a40f766947e1b471fae2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
