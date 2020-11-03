# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "c4df22538425ed2dc89483fe14f6ce22245df0e0",
        sha256 = "7a0ecbb05fa7c542a5c0e58e2c5bee8c16455a8ba3ca53936459f7e955a8967c",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
