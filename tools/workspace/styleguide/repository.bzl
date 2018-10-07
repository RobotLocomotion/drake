# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        # PR DRAFT: Change to upstream once PR is merged.
        commit = "ebd4c6915d52409700477cdbb57592c2abb567d8",
        sha256 = "45cd4deb5035e2910ca7f8205b358e2b93e03f9c0e5624711f2be534de118b8b",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
