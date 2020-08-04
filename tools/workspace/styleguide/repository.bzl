# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "47a6f403e7a6ed5e44e7875bcec4d27815063029",
        sha256 = "68344607a14230ede11b4dcf886ed9bef6df2c339f32ec724591a94e5d5f2e6b",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
