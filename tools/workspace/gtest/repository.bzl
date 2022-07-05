# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "release-1.12.1",
        sha256 = "81964fe578e9bd7c94dfdb09c8e4d6e6759e19967e397dbea48d1c10e45d0df2",  # noqa
        build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
        mirrors = mirrors,
    )
