# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "ddcc3fc705df4105a9dfd59efaa73c247c391cb5",
        sha256 = "31a06b925c81e194950f6bcb3c739a21ccbd40b9ef2da90feaa30741bdedcb72",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
