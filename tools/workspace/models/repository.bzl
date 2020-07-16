# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "e66c64b0b0bb4cde1eb5d164dada551d06909953",
        sha256 = "b6767a902bb4ea3e561d79f1c03b49464f525df16db076dab047354c991ee7ae",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
