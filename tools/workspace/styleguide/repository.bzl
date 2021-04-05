# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "748ed4dd4e543001421c6618646a60e2a2dca8ea",
        sha256 = "94b54d00f67a9d536dec685982b38f43b261eaf689a024209c40b39c567cbc02",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
