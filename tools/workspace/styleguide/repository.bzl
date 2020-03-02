# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "849a106b536d8dc5f504d59bc6b0446c608d57f7",
        sha256 = "553533e9f0159e54f7768e7655744a8a50dca931fa2fb01bcffc2725f843438e",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
