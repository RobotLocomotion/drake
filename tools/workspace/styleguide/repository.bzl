# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "2558302d96c2f2a3e04f64c2a46a4d32bd422d39",
        sha256 = "f2ec5dd818b77c63a2651ca7610d2b1c28750139f58c2360beb4032d35677217",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
