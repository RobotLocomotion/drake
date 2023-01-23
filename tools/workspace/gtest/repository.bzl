# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "v1.13.0",
        sha256 = "ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
