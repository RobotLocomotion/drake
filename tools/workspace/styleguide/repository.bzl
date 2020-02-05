# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "d612ab7a998a7edaeb8f5ceca498a81e611f4c9b",
        sha256 = "3e92bcbfea39457e0b18543765d5eddac6fae8a888226682e71eeff5d94c75bb",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
