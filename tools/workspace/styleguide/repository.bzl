# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(name):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "f9fb031554d398431bc0efcb511102d41bbed089",
        sha256 = "1e40f4595406e208de8bde66bc3425e6c0dce4ea96254cc2c7e4105316df9a31",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
    )
