# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "61b5b584781f3950b7489f557fc7ba2d073779a8",
        sha256 = "6629dbb1e0aee0ab17990ca9083a76b9a8e7aeff0db6297db5987a08b1ca8300",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
