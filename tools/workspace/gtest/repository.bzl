# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "release-1.12.0",
        sha256 = "2a4f11dce6188b256f3650061525d0fe352069e5c162452818efbbf8d0b5fe1c",  # noqa
        build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
        mirrors = mirrors,
    )
