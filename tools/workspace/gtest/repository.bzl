# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "release-1.8.1",
        sha256 = "9bf1fe5182a604b4135edc1a425ae356c9ad15e9b23f9f12a02e80184c3a249c",  # noqa
        build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
        mirrors = mirrors,
    )
