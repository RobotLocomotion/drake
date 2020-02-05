# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "4ed40237e2061fff76281fa0040e431c16871d6a",
        sha256 = "0ba3e0b651f0f8f57ca232cb68920224bad42114ed615884cae4180a6583e3bc",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
