# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "5d2a134bceadc432af30be16faf64755aa116ad3",
        sha256 = "0da2a66d25f1b9fb43e68f6ef3b67f41f2f7c67cf26e49edd7b0141cacefcd61",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
